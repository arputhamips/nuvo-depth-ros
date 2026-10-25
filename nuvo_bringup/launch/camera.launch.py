from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace',default_value='nuvo/front'),
        DeclareLaunchArgument('config',default_value=PathJoinSubstitution([FindPackageShare('nuvo_bringup'),'config','camera.yaml'])),
        Node(package='nuvo_camera',executable='nuvo_camera_node',namespace=LaunchConfiguration('namespace'),
             parameters=[LaunchConfiguration('config')],output='screen'),
    ])
