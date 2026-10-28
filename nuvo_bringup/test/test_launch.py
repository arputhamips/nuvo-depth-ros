"""Launch arguments retain YAML defaults and produce genuine typed ROS values."""
import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters, normalize_parameters

from nuvo_bringup.camera_launch import STARTUP_ARGUMENTS, typed_overrides


def test_omitted_arguments_preserve_yaml():
    assert typed_overrides({}) == {}
    assert typed_overrides({name: '' for name in STARTUP_ARGUMENTS}) == {}


def test_arguments_have_explicit_ros_types():
    overrides = typed_overrides({
        'calibration_file': '/calibrations/front: accepted.tar', 'cadence_hz': '7.5',
        'publish_depth': 'false', 'publish_points': 'TRUE',
        'publish_left_rectified': 'true', 'publish_right_rectified': 'false',
        'publish_disparity': 'false', 'allow_provisional_calibration': 'true',
    })
    assert evaluate_parameters(LaunchContext(), normalize_parameters([overrides]))[0] == {
        'calibration_file': '/calibrations/front: accepted.tar', 'cadence_hz': 7.5,
        'publish_depth': False, 'publish_points': True,
        'publish_left_rectified': True, 'publish_right_rectified': False,
        'publish_disparity': False, 'allow_provisional_calibration': True,
    }


@pytest.mark.parametrize('name,value', [
    ('publish_depth', '1'), ('publish_points', 'yes'), ('publish_disparity', 'maybe'),
    ('allow_provisional_calibration', 'false-ish'), ('cadence_hz', 'nan'),
    ('cadence_hz', 'inf'), ('cadence_hz', '0'), ('cadence_hz', '-1'), ('cadence_hz', 'fast'),
])
def test_invalid_arguments_fail_before_startup(name, value):
    with pytest.raises(ValueError, match=name):
        typed_overrides({name: value})


@pytest.mark.parametrize('filename', ['camera.launch.py', 'inspection.launch.py'])
def test_both_launches_expose_the_same_optional_controls(filename):
    path = Path(__file__).resolve().parents[1] / 'launch' / filename
    spec = importlib.util.spec_from_file_location('camera_launch_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    declarations = {action.name: action for action in description.entities
                    if isinstance(action, DeclareLaunchArgument)}
    assert set(STARTUP_ARGUMENTS).issubset(declarations)
    assert all(declarations[name].default_value[0].text == '' for name in STARTUP_ARGUMENTS)
    context = LaunchContext()
    context.launch_configurations.update({name: '' for name in STARTUP_ARGUMENTS})
    context.launch_configurations.update(name='sensor', namespace='robot/front', config='parameters.yaml')
    action = next(action for action in description.entities if isinstance(action, OpaqueFunction))
    assert isinstance(action.execute(context)[0], Node)
