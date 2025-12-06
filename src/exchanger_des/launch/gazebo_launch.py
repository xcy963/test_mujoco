import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from xacro import process_file
def generate_launch_description():
    # Get default path
    robot_name_in_model = "exchanger"
    pkg_path = get_package_share_directory('exchanger_des')
    # default_model_path = os.path.join(
    #     urdf_tutorial_path, 'urdf', 'exchanger.urdf')
    xacro_file = os.path.join(pkg_path, 'urdf', 'ros2_control.xacro')
    urdf_file = os.path.join(pkg_path, 'urdf', 'exchanger.urdf')

    robot_desc = process_file(xacro_file).toprettyxml(indent='  ')
    print(robot_desc)
    # with open(urdf_file, 'r') as f:
    #     robot_desc += f.read().replace('<?xml version="1.0"?>', '')
    # 在your_file.xacro中添加: <xacro:include filename="your_file.urdf" />
    # robot_desc = process_file(xacro_file).toprettyxml(indent='  ')
    # Read URDF file content
    # with open(default_model_path, 'r') as urdf_file:
    #     robot_description = urdf_file.read()

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}]
    )

    # Include another launch file for Gazebo
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'gazebo_ros'), '/launch', '/gazebo.launch.py']),
    )

    # Request Gazebo to spawn the robot
    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description',
                   '-entity', robot_name_in_model])

    package_name = 'exchanger_des'
    pkg_share = os.pathsep + os.path.join(get_package_prefix(package_name), 'share')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += pkg_share
    else:
        os.environ['GAZEBO_MODEL_PATH'] = "/usr/share/gazebo-11/models" + pkg_share

    return launch.LaunchDescription([
        robot_state_publisher_node,
        launch_gazebo,
        spawn_entity_node
    ])
