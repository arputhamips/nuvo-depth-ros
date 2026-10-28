"""Exercise host launcher selections without changing native or Docker state."""
import argparse
import copy
import importlib.util
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('run_camera', Path(__file__).resolve().parents[1] / 'tools/run_camera.py')
launcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launcher)


class HostLauncher(unittest.TestCase):
    def setUp(self):
        self.base = {'name': 'docker', 'services': {'camera': {
            'image': 'example:image', 'group_add': ['991'],
            'environment': {'ROS_DOMAIN_ID': '42'},
            'volumes': [{'type': 'bind', 'source': '/run/nuvo-depth',
                         'target': '/run/nuvo-depth', 'read_only': True}],
            'command': ['ros2', 'launch', 'nuvo_bringup', 'camera.launch.py',
                        'namespace:=robot/front', 'config:=/config/custom.yaml']}}}

    def configured(self, *arguments):
        return launcher.configure(copy.deepcopy(self.base), launcher.parser().parse_args(arguments))['services']['camera']

    def test_omission_preserves_user_yaml_namespace_domain_and_permissions(self):
        self.assertEqual(self.configured(), self.base['services']['camera'])

    def test_selected_products_disable_unselected_publishers_and_request_rate(self):
        service = self.configured('--outputs', 'points,right', '--cadence-hz', '7.5')
        command = service['command']
        for expected in ('publish_depth:=false', 'publish_points:=true', 'publish_left_rectified:=false',
                         'publish_right_rectified:=true', 'publish_disparity:=false', 'cadence_hz:=7.5'):
            self.assertIn(expected, command)
        self.assertFalse(any(item.startswith('allow_provisional_calibration:=') for item in command))

    def test_explicit_preview_and_domain_do_not_edit_original_document(self):
        service = self.configured('--allow-provisional-calibration', '--ros-domain-id', '93', '--namespace', 'robot/back')
        self.assertIn('allow_provisional_calibration:=true', service['command'])
        self.assertNotIn('namespace:=robot/front', service['command'])
        self.assertEqual(service['environment']['ROS_DOMAIN_ID'], '93')
        self.assertEqual(self.base['services']['camera']['environment']['ROS_DOMAIN_ID'], '42')

    def test_calibration_mounts_exact_bundle_readonly_without_host_config_write_access(self):
        with tempfile.TemporaryDirectory(prefix='nuvo bundle ') as directory:
            bundle = Path(directory) / 'my calibration.tar'
            bundle.write_bytes(b'test')
            service = self.configured('--calibration-file', str(bundle))
            self.assertIn('calibration_file:=/calibration/bundle', service['command'])
            self.assertEqual(service['volumes'][-1]['source'], str(bundle.resolve()))
            self.assertTrue(all(v['read_only'] for v in service['volumes']))
            self.assertFalse(service['volumes'][-1]['bind']['create_host_path'])
            self.assertNotIn('privileged', service)
            self.assertEqual(bundle.read_bytes(), b'test')

    def test_custom_config_uses_separate_file_mount_and_removes_old_argument(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / 'custom.yaml'
            config.write_text('unchanged')
            service = self.configured('--config', str(config))
            self.assertNotIn('config:=/config/custom.yaml', service['command'])
            self.assertIn('config:=/nuvo-parameters.yaml', service['command'])
            self.assertEqual(config.read_text(), 'unchanged')

    def test_missing_files_fail_before_container_start(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(FileNotFoundError):
                self.configured('--calibration-file', str(Path(directory) / 'missing'))

    def test_private_bundle_is_snapshotted_for_unprivileged_restarts_without_extra_files(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bundle = root / 'bundle'
            bundle.mkdir()
            for name in launcher.BUNDLE_FILES:
                (bundle / name).write_bytes(name.encode())
                (bundle / name).chmod(0o600)
            (bundle / 'private-key.pem').write_text('must not copy')
            target = launcher.snapshot_input(bundle, root / 'snapshots', bundle=True)
            self.assertEqual(set(p.name for p in target.iterdir()), set(launcher.BUNDLE_FILES))
            self.assertEqual(launcher.snapshot_input(bundle, root / 'snapshots', bundle=True), target)
            self.assertEqual((bundle / 'manifest.json').read_bytes(), b'manifest.json')
            if launcher.sys.platform == 'linux':
                self.assertEqual((bundle / 'manifest.json').stat().st_mode & 0o777, 0o600)
                self.assertEqual((target / 'manifest.json').stat().st_mode & 0o777, 0o440)
            self.assertNotEqual(target, bundle)

    def test_oversized_input_is_not_snapshotted(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = root / 'huge.yaml'
            config.write_bytes(b'x' * (1024 * 1024 + 1))
            with self.assertRaises(ValueError):
                launcher.snapshot_input(config, root / 'snapshots')
            self.assertFalse((root / 'snapshots').exists())

    @unittest.skipUnless(launcher.sys.platform == 'linux', 'Linux symlink permissions')
    def test_snapshot_cache_rejects_redirected_directories(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / 'input.yaml'
            source.write_text('camera')
            outside = root / 'outside'
            outside.mkdir()
            cache = root / 'cache'
            cache.symlink_to(outside, target_is_directory=True)
            with self.assertRaises(ValueError):
                launcher.snapshot_input(source, cache)
            self.assertEqual(list(outside.iterdir()), [])

    def test_invalid_cadence_and_outputs_are_rejected(self):
        for value in ('nan', 'inf', '-1', '0', '0.0001', '61'):
            with self.subTest(value=value), self.assertRaises(argparse.ArgumentTypeError):
                launcher.cadence(value)
        for value in ('', 'rgb', 'depth,,points'):
            with self.subTest(value=value), self.assertRaises(argparse.ArgumentTypeError):
                launcher.output_selection(value)


if __name__ == '__main__':
    unittest.main()
