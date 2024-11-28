from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    robot_namespace = DeclareLaunchArgument(
        'robot_namespace',
        default_value='',
        description='Namespace for the robot'
    )

    world_file = PathJoinSubstitution(
        [FindPackageShare('turtlebot3_race'), 'worlds', 'hexagonal_track.world']
    )
    
    model_file = PathJoinSubstitution(
        [FindPackageShare('turtlebot3_race'), 'models', 'turtlebot3_waffle','turtlebot3_waffle.urdf.xacro']
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py']
        )]),
        launch_arguments={'world': world_file}.items()
    )

    turtlebot1_spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_turtlebot', '-file', model_file, '-x', '1.6', '-y', '2.4', '-z', '0',
                   '-robot_namespace', 'my_turtlebot'],
        output='screen'
    )

    turtlebot2_spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'opp_turtlebot', '-file', model_file, '-x', '1.27', '-y', '3.0', '-z', '0',
                   '-robot_namespace', 'opp_turtlebot'],
        output='screen'
    )

    turtlebot1_control = Node(
        package='turtlebot3_race',
        executable='turtlebot_controller',
        name='turtlebot1_controller',
        namespace='my_turtlebot',
        output='screen'
    )

    turtlebot2_control = Node(
        package='turtlebot3_race',
        executable='turtlebot_controller',
        name='turtlebot2_controller',
        namespace='opp_turtlebot',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        turtlebot1_spawn,
        turtlebot2_spawn,
        #turtlebot1_control,
        #turtlebot2_control,
    ])

