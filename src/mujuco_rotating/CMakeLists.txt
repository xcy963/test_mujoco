cmake_minimum_required(VERSION 3.8)
project(mujuco_rotating)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# add_compile_options(//开优化
#      -O3 
# )
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Python3 COMPONENTS Development NumPy)
# find_package(lodepng REQUIRED)
find_package(glfw3  REQUIRED)
find_package(OpenGL REQUIRED)
find_package(OpenCV REQUIRED)   
find_package(Eigen3 REQUIRED)


set(MUJOCO_HOME "../../mujoco-3.3.7")
set(MUJOCO_SIMULATE_DIR "${MUJOCO_HOME}/simulate")
find_path(MUJOCO_INCLUDE_DIRS
  NAMES mujoco/mujoco.h
  PATHS ${MUJOCO_HOME}/include
  NO_DEFAULT_PATH # 只在指定路径中查找
)

find_library(MUJOCO_LIBRARIES
  NAMES mujoco
  PATHS ${MUJOCO_HOME}/lib
  NO_DEFAULT_PATH
)

if(NOT MUJOCO_INCLUDE_DIRS)
  message(FATAL_ERROR "Failed to find MuJoCo headers at ${MUJOCO_HOME}/include")
endif()
if(NOT MUJOCO_LIBRARIES)
  message(FATAL_ERROR "Failed to find libmujoco.so at ${MUJOCO_HOME}/lib")
endif()

message(WARNING "我们手动设置mujuco的头文件还有库地址")
message(WARNING "Found MuJoCo include at: ${MUJOCO_INCLUDE_DIRS}")
message(WARNING "Found MuJoCo library: ${MUJOCO_LIBRARIES}")


set(SIMULATE_SOURCES
  ${MUJOCO_SIMULATE_DIR}/glfw_adapter.cc
  ${MUJOCO_SIMULATE_DIR}/glfw_dispatch.cc
  ${MUJOCO_SIMULATE_DIR}/platform_ui_adapter.cc
  ${MUJOCO_SIMULATE_DIR}/simulate.cc
)
# set(OTHER_SOURCES
#   # include/mujuco_rotating_node.cpp
#   # include/mujoco_test/mujoco_simulator.cpp
# )

add_library(mujoco_sim 
  ${SIMULATE_SOURCES}
)

target_include_directories(mujoco_sim PUBLIC
  ${MUJOCO_SIMULATE_DIR}
  ${MUJOCO_INCLUDE_DIRS}
  # ${EGL_INCLUDE_DIRS}
)
target_link_libraries(mujoco_sim PUBLIC
    glfw
  OpenGL::GL
  # ${EGL_LIBRARIES}
)

#到这里mujuco应该结束了

add_executable(mujuco_rotating_node
  src/main.cpp
  # ${SIMULATE_SOURCES}
  # ${OTHER_SOURCES}
)

target_include_directories(mujuco_rotating_node PUBLIC
  include
)
# 为目标链接库
target_link_libraries(mujuco_rotating_node
  ${rclcpp_LIBRARIES}
  ${MUJOCO_LIBRARIES} 

  Python3::Python

  lodepng
  ${OpenCV_LIBS} 
  mujoco_sim
  Eigen3::Eigen

)

# 声明ROS2相关的依赖，确保它们出现在头文件和链接路径中
ament_target_dependencies(mujuco_rotating_node
  rclcpp
  std_msgs
  sensor_msgs
)

# 安装可执行文件到指定目录
install(TARGETS
  mujuco_rotating_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件和config文件
install(DIRECTORY
  launch 
  engineer_module
  DESTINATION share/${PROJECT_NAME}/
)




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
