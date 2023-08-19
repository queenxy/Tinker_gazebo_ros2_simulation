import os

from sympy import O

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    robot_bring_up = get_package_share_directory('tinker_sim')
    robot_launch_dir = os.path.join(robot_bring_up, 'launch')
    local_bringup_dir = get_package_share_directory('navigation_sim')

    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    start_rviz = LaunchConfiguration("start_rviz") 
    use_teleop = LaunchConfiguration("use_teleop")

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='True',
        description='Whether to execute gzclient')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        # TODO(orduno) Switch back once ROS argument passing has been fixed upstream
        #              https://github.com/ROBOTIS-GIT/turtlebot3_simulations/issues/91
        # default_value=os.path.join(get_package_share_directory('turtlebot3_gazebo'),
        # worlds/turtlebot3_worlds/waffle.model')
        default_value=os.path.join(local_bringup_dir, 'worlds', 'test_world_simple.world'),
        description='Full path to world model file to load')

    declare_rviz_cmd = DeclareLaunchArgument(
        'start_rviz',
        default_value='True',
        description='Start RViz2 automatically with this launch file.')

    declare_teleop_cmd = DeclareLaunchArgument(
        'use_teleop',
        default_value='True',
        description='Use teleop use control Tinker')

    declared_arguments = [
        declare_simulator_cmd,
        declare_world_cmd,
        declare_rviz_cmd,
        declare_teleop_cmd
    ]

    
    
    # Localization and SLAM
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    localization_params_file = LaunchConfiguration('localization_params_file')
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(local_bringup_dir,
                                   'params', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    declare_localization_params_file_cmd = DeclareLaunchArgument(
        'localization_params_file',
        default_value=os.path.join(local_bringup_dir,
                                   'params', 'ekf.yaml'),
        description='Full path to the ROS2 parameters file to use for the localization')

    declare_rviz_config_file = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(local_bringup_dir,
                                   'rviz', 'nav.rviz'),
        description='Full path to the rviz config file')


    nav_arguments = [
        declare_use_sim_time_argument, 
        declare_localization_params_file_cmd,
        declare_slam_params_file_cmd,
        declare_rviz_config_file
    ]
    # Localization is used to publish tf2 message
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[
           localization_params_file,
           {'use_sim_time': use_sim_time}
       ]
    )
    start_async_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    # Robot
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_launch_dir, 'gazebo_sim.launch.py')),
        launch_arguments={
            "headless": headless,
            "world": world,
            "start_rviz": start_rviz,
            "use_teleop": use_teleop,
            "rviz_config_file": rviz_config_file
        }.items())
    
    nodes = [bringup_cmd, robot_localization_node, start_async_slam_toolbox_node]
    return LaunchDescription(declared_arguments + nav_arguments + nodes)