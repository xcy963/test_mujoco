from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('engineer_26arm')
    urdf_path = os.path.join(pkg_share, 'urdf', 'openarm_single_control.urdf')
    # dynamic_params = {
    #     'paramfs_path': os.path.join(pkg_share, 'config', 'param.yaml'),
    #     'configfs_path': os.path.join(pkg_share, 'config', 'config.yaml')
    # }
    # print(config_path+"我还在launch文件里面")
    # moveit_config = MoveItConfigsBuilder("final_car", package_name="final_des").robot_description_kinematics(file_path="config/kinematics.yaml").to_moveit_configs()
    # moveit_config_params = moveit_config.to_dict()
    return LaunchDescription([
        Node(
            package='test_seven_dofik',
            name='test_seven_dofik',#把节点注册成这个,可以和代码里面指定的不一样，没有研究过这个机制
            executable='test_seven_dofik',
            output='screen',
            parameters=[
                {
                    "URDF_PATH": urdf_path
                # "/home/hitcrt/enginner_26/test_mujoco_ros/src/test_mujoco/modules/trs_so_arm100/scene.xml

                }
                # moveit_config_params
            ],
            arguments=['--ros-args', '--log-level', 'info'],
            emulate_tty=True,
        )
    ])