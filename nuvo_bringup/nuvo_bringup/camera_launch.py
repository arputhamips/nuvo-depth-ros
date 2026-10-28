"""Typed optional launch overrides shared by normal and inspection startup."""
import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


STARTUP_ARGUMENTS = {
    'calibration_file': (str, 'Accepted native calibration bundle directory or uncompressed .tar; local source only.'),
    'cadence_hz': (float, 'Upper target delivery rate in Hz; does not change the native capture trigger.'),
    'publish_depth': (bool, 'Publish depth, validity and depth CameraInfo.'),
    'publish_points': (bool, 'Enable XYZ output; generated only while points has subscribers.'),
    'publish_left_rectified': (bool, 'Enable left rectified image and CameraInfo.'),
    'publish_right_rectified': (bool, 'Enable right rectified image and CameraInfo.'),
    'publish_disparity': (bool, 'Enable matching-grid disparity.'),
    'allow_provisional_calibration': (bool, 'Explicitly allow provisional calibration; diagnostics remain unqualified.'),
}


def typed_overrides(values):
    """Empty launch arguments preserve the value from the selected YAML."""
    result = {}
    for name, (kind, _) in STARTUP_ARGUMENTS.items():
        value = values.get(name, '')
        if value == '':
            continue
        if kind is bool:
            if value.lower() not in ('true', 'false'):
                raise ValueError(f'{name} must be true or false')
            result[name] = value.lower() == 'true'
        elif kind is float:
            try:
                number = float(value)
            except ValueError as error:
                raise ValueError(f'{name} must be a finite positive number') from error
            if not math.isfinite(number) or number <= 0:
                raise ValueError(f'{name} must be a finite positive number')
            result[name] = number
        else:
            result[name] = ParameterValue(value, value_type=str)
    return result


def camera_launch_description(namespace='nuvo/front', config='camera.yaml', inspection=False):
    def launch_camera(context):
        overrides = typed_overrides({
            name: LaunchConfiguration(name).perform(context) for name in STARTUP_ARGUMENTS
        })
        return [Node(
            package='nuvo_camera', executable='nuvo_camera_node',
            name=LaunchConfiguration('name'), namespace=LaunchConfiguration('namespace'),
            parameters=[LaunchConfiguration('config'), overrides],
            remappings=[('/diagnostics', 'diagnostics'), ('/clock', 'clock')] if inspection else [],
            output='screen')]

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=namespace,
                              description='Namespace for measurement topics.'),
        DeclareLaunchArgument('name', default_value='nuvo_camera', description='ROS node name.'),
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([
            FindPackageShare('nuvo_bringup'), 'config', config]),
            description='Driver parameters YAML; frame IDs are set here.'),
        *[DeclareLaunchArgument(name, default_value='', description=description +
                                ' Empty preserves the configured YAML value.')
          for name, (_, description) in STARTUP_ARGUMENTS.items()],
        OpaqueFunction(function=launch_camera),
    ])
