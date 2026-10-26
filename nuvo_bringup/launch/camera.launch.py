from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='nuvo/front',
                              description='Namespace for measurement topics.'),
        DeclareLaunchArgument('name', default_value='nuvo_camera',
                              description='ROS node name.'),
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([
            FindPackageShare('nuvo_bringup'), 'config', 'camera.yaml']),
            description='Driver parameters YAML; frame IDs are set here.'),
        Node(package='nuvo_camera', executable='nuvo_camera_node',
             name=LaunchConfiguration('name'),
             namespace=LaunchConfiguration('namespace'),
             parameters=[LaunchConfiguration('config')], output='screen'),
    ])
