import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ass_pkg_name = 'erl_ass2_pkg'
    ass_pkg_share = get_package_share_directory('erl_ass2_pkg')
    namespace = LaunchConfiguration('planning_problem')
    
    declare_namespace_cmd = DeclareLaunchArgument(
        'planning_problem',
        default_value='',
        description='Namespace')
    
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')
    
    problem_file_path = ass_pkg_share+'/pddl/ass2_problem.pddl'
    with open(problem_file_path, 'r') as f:
        problem_file_contents = " ".join(line.strip() for line in f.readlines())
    
    load_problem_node = Node(
        package=ass_pkg_name,
        executable='load_problem',
        name='load_problem_node',
        namespace=namespace,
        output='screen',
        arguments=[problem_file_contents]
    )
    
    ## action nodes
    move_cmd = Node(
        package='erl_ass2_pkg',
        executable='move_node',
        name='move_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    search_cmd = Node(
        package='erl_ass2_pkg',
        executable='search_node',
        name='search_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    find_marker_cmd = Node(
        package='erl_ass2_pkg',
        executable='find_marker_node',
        name='find_marker_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    end_mission_cmd = Node(
        package='erl_ass2_pkg',
        executable='end_mission_node',
        name='end_mission_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    return LaunchDescription([
        declare_namespace_cmd,
        stdout_linebuf_envvar,
        
        
        load_problem_node,
        move_cmd,
        search_cmd,
        find_marker_cmd,
        end_mission_cmd
    ])
    