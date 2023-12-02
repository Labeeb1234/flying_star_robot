from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('fs_bot_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'fs_bot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml() 

    world_file = 'sample_world.world'
    world_file_path = os.path.join(share_dir, 'worlds', world_file)


    world = LaunchConfiguration('world')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')

    world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_file_path,
        description='Full path to the world model file to load'
    )



    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'pause': 'true',
            'world': world
        }.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'fs_bot',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    # service node 
    transform_service_node = Node(
        package='bot_transform',
        executable='transform',
        name='bot_tranform_service'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        world_cmd,
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
        transform_service_node
    ])
