import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
# from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_description = get_package_share_directory('engineer_26arm') #返回的是/home/hitcrt/engineer_arm/install/final_car/share/final_car这个

    urdf_path = os.path.join(pkg_description, "urdf", 'openarm_single_control.urdf')
    rviz_path = os.path.join(pkg_description, "urdf", 'default.rviz')
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
    rsp_params = {'robot_description': robot_desc}
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[rsp_params]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_path],
    )
 
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                             description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=urdf_path,
                                             description='Absolute path to robot urdf file'),
        
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
