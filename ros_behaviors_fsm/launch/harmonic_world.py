import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import EnvironmentVariable


def generate_launch_description():
    world_file_name = LaunchConfiguration('world_file')
    package_share = get_package_share_directory('ros_behaviors_fsm')
    neato_share = get_package_share_directory('neato2_gazebo')
    world = PathJoinSubstitution([package_share, 'worlds', world_file_name])
    model_path = os.path.join(neato_share, 'models')
    gz_sim_share = get_package_share_directory('ros_gz_sim')

    bridge_topics = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        '/world/gauntlet_harmonic/model/neato_standalone/link/base_link/sensor/bumpers/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
        '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        '/model/neato_standalone/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('world_file'),
        DeclareLaunchArgument('gz_flags', default_value='-r'),
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[
                model_path + ':',
                EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gz_sim_share, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': [world, ' ', LaunchConfiguration('gz_flags')]}.items(),
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=bridge_topics,
            remappings=[('/model/neato_standalone/tf', '/tf'),
                        ('/world/gauntlet_harmonic/model/neato_standalone/link/base_link/sensor/bumpers/contact', '/bumper')],
            output='screen',
        ),
        Node(
            package='neato_node2',
            executable='simulator_adapter',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='fix_scan',
            executable='scan_to_pc2',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(neato_share, 'launch', 'robot_state_publisher.py')
            ),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),
    ])
