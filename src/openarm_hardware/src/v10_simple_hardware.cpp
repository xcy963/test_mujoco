// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "openarm_hardware/v10_simple_hardware.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openarm_hardware {

OpenArm_v10HW::OpenArm_v10HW() = default;

bool OpenArm_v10HW::parse_config(const hardware_interface::HardwareInfo& info) {
  // Parse CAN interface (default: can0)
  auto it = info.hardware_parameters.find("can_interface");
  can_interface_ = (it != info.hardware_parameters.end()) ? it->second : "can0";

  // Parse arm prefix (default: empty for single arm, "left_" or "right_" for
  // bimanual)
  it = info.hardware_parameters.find("arm_prefix");
  arm_prefix_ = (it != info.hardware_parameters.end()) ? it->second : "";

  // Parse gripper enable (default: true for V10)
  it = info.hardware_parameters.find("hand");
  if (it == info.hardware_parameters.end()) {
    hand_ = true;  // Default to true for V10
  } else {
    // Handle both "true"/"True" and "false"/"False"
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    hand_ = (value == "true");
  }

  // Parse CAN-FD enable (default: true for V10)
  it = info.hardware_parameters.find("can_fd");
  if (it == info.hardware_parameters.end()) {
    can_fd_ = true;  // Default to true for V10
  } else {
    // Handle both "true"/"True" and "false"/"False"
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    can_fd_ = (value == "true");
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Configuration: CAN=%s, arm_prefix=%s, hand=%s, can_fd=%s",
              can_interface_.c_str(), arm_prefix_.c_str(),
              hand_ ? "enabled" : "disabled", can_fd_ ? "enabled" : "disabled");
  return true;
}

void OpenArm_v10HW::generate_joint_names() {
  joint_names_.clear();
  // TODO: read from urdf properly and sort in the future.
  // Currently, the joint names are hardcoded for order consistency to align
  // with hardware. Generate arm joint names: openarm_{arm_prefix}joint{N}
  for (size_t i = 1; i <= ARM_DOF; ++i) {
    std::string joint_name =
        "openarm_" + arm_prefix_ + "joint" + std::to_string(i);
    joint_names_.push_back(joint_name);
  }

  // Generate gripper joint name if enabled
  if (hand_) {
    std::string gripper_joint_name = "openarm_" + arm_prefix_ + "finger_joint1";
    joint_names_.push_back(gripper_joint_name);
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Added gripper joint: %s",
                gripper_joint_name.c_str());
  } else {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "Gripper joint NOT added because hand_=false");
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Generated %zu joint names for arm prefix '%s'",
              joint_names_.size(), arm_prefix_.c_str());
}

hardware_interface::CallbackReturn OpenArm_v10HW::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  // Parse configuration
  if (!parse_config(info)) {
    return CallbackReturn::ERROR;
  }

  // Generate joint names based on arm prefix
  generate_joint_names();

  // Validate joint count (7 arm joints + optional gripper)
  size_t expected_joints = ARM_DOF + (hand_ ? 1 : 0);
  if (joint_names_.size() != expected_joints) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                 "Generated %zu joint names, expected %zu", joint_names_.size(),
                 expected_joints);
    return CallbackReturn::ERROR;
  }

  // Initialize OpenArm with configurable CAN-FD setting
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Initializing OpenArm on %s with CAN-FD %s...",
              can_interface_.c_str(), can_fd_ ? "enabled" : "disabled");
  // openarm_ =
  //     std::make_unique<openarm::can::socket::OpenArm>(can_interface_, can_fd_);

  // // Initialize arm motors with V10 defaults
  // openarm_->init_arm_motors(DEFAULT_MOTOR_TYPES, DEFAULT_SEND_CAN_IDS,
  //                           DEFAULT_RECV_CAN_IDS);

  // // Initialize gripper if enabled
  // if (hand_) {
  //   RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Initializing gripper...");
  //   openarm_->init_gripper_motor(DEFAULT_GRIPPER_MOTOR_TYPE,
  //                                DEFAULT_GRIPPER_SEND_CAN_ID,
  //                                DEFAULT_GRIPPER_RECV_CAN_ID);
  // }

  // Initialize state and command vectors based on generated joint count
  const size_t total_joints = joint_names_.size();
  pos_commands_.resize(total_joints, 0.0);
  vel_commands_.resize(total_joints, 0.0);
  tau_commands_.resize(total_joints, 0.0);
  pos_states_.resize(total_joints, 0.0);
  vel_states_.resize(total_joints, 0.0);
  tau_states_.resize(total_joints, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "OpenArm V10 Simple HW initialized successfully");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArm_v10HW::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Set callback mode to ignore during configuration
  // openarm_->refresh_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // openarm_->recv_all();

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
OpenArm_v10HW::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
OpenArm_v10HW::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // TODO: consider exposing only needed interfaces to avoid undefined behavior.
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION,
        &pos_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY,
        &vel_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn OpenArm_v10HW::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {

  //HACK第一步,创建一个节点,让他能接受我们的消息
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Activating OpenArm V10...HACK BY XCY NO CAN!!!");
  // 创建一个 node（和 controller_manager 在同一进程）
  node_ = rclcpp::Node::make_shared("openarm_v10_hw_service_node");

  // 创建 service
  srv_ = node_->create_service<arm_msg::srv::SetJointPositions>(
      "openarm/set_joint_positions",
      std::bind(&OpenArm_v10HW::set_joint_positions_cb, this,
                std::placeholders::_1, std::placeholders::_2));

  // 起一个单独线程跑 executor
  executor_thread_ = std::thread([this]() {
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node_);
    exec.spin();
  });

  // 初始化状态（可选）
  std::fill(pos_states_.begin(), pos_states_.end(), 0.0);
  std::fill(vel_states_.begin(), vel_states_.end(), 0.0);
  std::fill(tau_states_.begin(), tau_states_.end(), 0.0);

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "OpenArm V10 activated, waiting for service calls");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArm_v10HW::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  //HACK第二步,把deactivate改调
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Deactivating OpenArm V10...");
  if (node_) {
      rclcpp::shutdown();
  }
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
  node_.reset();
  srv_.reset();

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type OpenArm_v10HW::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArm_v10HW::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // std::lock_guard<std::mutex> lock(cmd_mutex_);//如果要用记得加锁

  // 这里用 pos_commands_ 做你原来的控制：
  // - 如果是 NO-CAN 假驱动，可以什么都不做
  // - 如果还在用 CAN，就照你原来的方式构造 MITParam 等

  // 例：假驱动什么都不干
  return hardware_interface::return_type::OK;
}

void OpenArm_v10HW::return_to_zero() {
return;
}

// Gripper mapping helper functions
double OpenArm_v10HW::joint_to_motor_radians(double joint_value) {
  // Joint 0=closed -> motor 0 rad, Joint 0.044=open -> motor -1.0472 rad
  return (joint_value / GRIPPER_JOINT_0_POSITION) *
         GRIPPER_MOTOR_1_RADIANS;  // Scale from 0-0.044 to 0 to -1.0472
}

double OpenArm_v10HW::motor_radians_to_joint(double motor_radians) {
  // Motor 0 rad=closed -> joint 0, Motor -1.0472 rad=open -> joint 0.044
  return GRIPPER_JOINT_0_POSITION *
         (motor_radians /
          GRIPPER_MOTOR_1_RADIANS);  // Scale from 0 to -1.0472 to 0-0.044
}

void OpenArm_v10HW::set_joint_positions_cb(
    const std::shared_ptr<arm_msg::srv::SetJointPositions::Request> req,
    std::shared_ptr<arm_msg::srv::SetJointPositions::Response> resp)
{
  std::lock_guard<std::mutex> lock(cmd_mutex_);

  if (req->positions.size() != joint_names_.size()) {
    resp->success = false;
    resp->message = "positions size (" + std::to_string(req->positions.size()) +
                    ") != joint count (" +
                    std::to_string(joint_names_.size()) + ")";
    RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                "%s", resp->message.c_str());
    return;
  }

  // 把请求的角度写到 pos_commands_
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    pos_commands_[i] = req->positions[i];
  }

  resp->success = true;
  resp->message = "Set joint positions success";
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Got new joint targets via service");
}

}  // namespace openarm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(openarm_hardware::OpenArm_v10HW,
                       hardware_interface::SystemInterface)
