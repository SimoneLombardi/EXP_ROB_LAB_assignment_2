"""
Spawn Robot Description
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction


def generate_launch_description():
    test_robot_description_share = FindPackageShare(package='robot_urdf').find('robot_urdf')
    default_model_path = os.path.join(test_robot_description_share, 'urdf/robot5.xacro')
    rviz_config_path = os.path.join(test_robot_description_share, 'config/rviz_cfg.rviz')
    wordl_file_path = os.path.join(test_robot_description_share, 'worlds/world_ass2_noRobot.world')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}],
        #{'use_sim_time':True}],
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        #parameters=[{'use_sim_time':True}],
    )
    
    aruco_node = Node(
        package='ros2_aruco',
        executable='aruco_node',
        name='aruco_node',
        output='screen',
    )

    # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', 'my_test_robot', '-topic', '/robot_description', '-x', '2.0', '-y', '2.0'],
        output='screen',
        )
    
    rviz_launch = ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_path],
            output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                                    description='Absolute path to robot urdf file'),
        ExecuteProcess(
            cmd=['gzserver', '--verbose', wordl_file_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'),
        
        spawn_entity,
        robot_state_publisher_node,
        joint_state_publisher_node,
        aruco_node,
        
        TimerAction(
            period=3.0,
            actions=[rviz_launch]),
    ])
