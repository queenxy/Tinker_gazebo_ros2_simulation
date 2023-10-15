from platform import node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    robot_bringup_dir = get_package_share_directory('tinker_sim')

    # Declare arguments
    start_rviz = LaunchConfiguration("start_rviz") 
    use_teleop = LaunchConfiguration("use_teleop")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    ur_initial_joint_controller = LaunchConfiguration("ur_initial_joint_controller")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")

    declare_rviz_cmd = DeclareLaunchArgument(
        'start_rviz',
        default_value='False',
        description='Start RViz2 automatically with this launch file.')

    declare_teleop_cmd = DeclareLaunchArgument(
        'use_teleop',
        default_value='False',
        description='Use teleop use control Tinker base')

    declare_rviz_config_file = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(robot_bringup_dir,
                                   'rviz', 'tinker_config.rviz'),
        description='Full path to the rviz config file')
    
    declare_ur_initial_joint_controller = DeclareLaunchArgument(
            "ur_initial_joint_controller",
            default_value="joint_trajectory_controller",
            description="Robot controller to start.",
        )
    
    declare_description_package = DeclareLaunchArgument(
            "description_package",
            default_value="tinker_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )

    declare_description_file = DeclareLaunchArgument(
            "description_file",
            default_value="tinker_gazebo.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    
    declared_arguments = [
        declare_rviz_cmd,
        declare_teleop_cmd,
        declare_rviz_config_file,
        declare_ur_initial_joint_controller,
        declare_description_file,
        declare_description_package,
    ]


    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "use_gazebo_classic:=true",
            " ",
            "sim_gazebo:=true",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "tinker_robot"],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    tinker_nav_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["tinker_controller", "--controller-manager", "/controller_manager"],
    )

    ur_joint_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[ur_initial_joint_controller, "-c", "/controller_manager"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(start_rviz),
    )

    teleop_node= Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop",
        prefix = 'gnome-terminal --',
        remappings=[   
            ('/cmd_vel','/tinker_controller/cmd_vel_unstamped')
            ],
        condition=IfCondition(use_teleop),
    )

    nodes = [
        gazebo,
        robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster_spawner,
        ur_joint_controller_spawner,
        tinker_nav_controller_spawner,
        rviz_node,
        teleop_node
    ]

    return LaunchDescription(declared_arguments + nodes)