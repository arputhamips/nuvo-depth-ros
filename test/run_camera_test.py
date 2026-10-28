"""Exercise host launcher selections without changing native or Docker state."""
import argparse
import copy
from contextlib import ExitStack, redirect_stderr, redirect_stdout
import importlib.util
import io
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

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


@unittest.skipUnless(sys.platform == 'linux' and os.geteuid() == 0,
                     'Root-owned boot-unit contracts run in the Linux CI container')
class HostNetwork(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='nuvo host network ')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.proc = self.root / 'rmem_max'
        self.proc.write_text('212992\n')
        self.untouched = {name: self.root / name for name in ('rmem_default', 'wmem_max', 'wmem_default')}
        for path in self.untouched.values():
            path.write_text('212992\n')
        self.unit = self.root / 'nuvo-depth-ros-network.service'
        self.commands = []
        self.fail_command = None
        self.execute_unit = True
        stack = ExitStack()
        self.addCleanup(stack.close)
        stack.enter_context(mock.patch.object(launcher, 'RMEM_MAX', self.proc))
        stack.enter_context(mock.patch.object(launcher, 'DDS_HOST_UNIT', self.unit))
        stack.enter_context(mock.patch.object(launcher, 'SYSTEMD_UNIT_DIRS',
                                             (self.root / 'runtime-units', self.root / 'vendor-units')))
        stack.enter_context(mock.patch.object(launcher, 'run', side_effect=self.fake_run))
        self.stderr = io.StringIO()
        stack.enter_context(redirect_stderr(self.stderr))

    def fake_run(self, command, **kwargs):
        self.commands.append(command)
        if self.fail_command and command[:len(self.fail_command)] == self.fail_command:
            raise subprocess.CalledProcessError(1, command)
        if len(command) > 1 and Path(command[1]).name == 'check_native.py':
            print('Read-only native readiness passed', file=kwargs.get('stdout') or sys.stdout)
        if command[:2] == ['systemctl', 'restart'] and self.execute_unit:
            # Execute the exact boot action against a temporary proc fixture.
            line = next(line for line in self.unit.read_text().splitlines() if line.startswith('ExecStart='))
            executable, option, code = shlex.split(line.removeprefix('ExecStart='))
            self.assertEqual((executable, option), ('/usr/bin/python3', '-c'))
            subprocess.check_call([sys.executable, '-c', code])
        if command[-3:] == ['config', '--format', 'json']:
            return SimpleNamespace(stdout=json.dumps({'name': 'docker', 'services': {'camera': {
                'image': 'example:tested', 'command': ['ros2', 'launch', 'nuvo_bringup', 'camera.launch.py'],
                'volumes': [], 'environment': {}}}}))
        return SimpleNamespace(stdout='', returncode=0)

    def assert_defaults_unchanged(self):
        for path in self.untouched.values():
            self.assertEqual(path.read_text(), '212992\n')

    def invoke_main(self, *arguments):
        original_run = subprocess.run

        def inspect(command, **kwargs):
            if command[:3] == ['docker', 'image', 'inspect']:
                return SimpleNamespace(returncode=0, stdout=json.dumps([{
                    'Config': {'Labels': {launcher.CONTROLS_LABEL: '1'}}}]))
            return original_run(command, **kwargs)

        with mock.patch('grp.getgrnam', return_value=SimpleNamespace(gr_gid=991)), \
                mock.patch.object(launcher.subprocess, 'run', side_effect=inspect), \
                redirect_stdout(io.StringIO()) as output:
            result = launcher.main(list(arguments))
        return result, output.getvalue()

    def test_low_limit_is_raised_and_persisted_without_changing_defaults(self):
        launcher.prepare_host_network()
        self.assertEqual(int(self.proc.read_text()), 8388608)
        text = self.unit.read_text()
        self.assertIn('After=systemd-sysctl.service\nBefore=docker.service', text)
        self.assertIn('WantedBy=multi-user.target', text)
        self.assertNotIn(str(launcher.ROOT), text)
        self.assertEqual(self.unit.stat().st_mode & 0o777, 0o644)
        self.assertEqual(self.unit.stat().st_uid, 0)
        self.assert_defaults_unchanged()

    def test_equal_higher_and_repeated_setup_do_not_lower_or_rewrite_limits_or_unit(self):
        launcher.prepare_host_network()
        modified = self.unit.stat().st_mtime_ns
        for value in (launcher.DDS_RECEIVE_MINIMUM, launcher.DDS_RECEIVE_MINIMUM * 4):
            self.proc.write_text(str(value) + '\n')
            before = self.proc.stat().st_mtime_ns
            launcher.prepare_host_network()
            self.assertEqual(self.proc.read_text(), str(value) + '\n')
            self.assertEqual(self.proc.stat().st_mtime_ns, before)
            self.assertEqual(self.unit.stat().st_mtime_ns, modified)
        self.assert_defaults_unchanged()

    def test_exact_boot_action_preserves_higher_admin_value_and_repairs_low_boot_value(self):
        launcher.prepare_host_network()
        for value in (launcher.DDS_RECEIVE_MINIMUM * 4, 212992):
            self.proc.write_text(str(value))
            self.fake_run(['systemctl', 'restart', self.unit.name])
            self.assertEqual(int(self.proc.read_text()), max(value, launcher.DDS_RECEIVE_MINIMUM))

    def test_dry_run_has_no_writes_or_systemctl_and_does_not_require_root(self):
        before = {path.name: path.read_bytes() for path in self.root.iterdir()}
        with mock.patch.object(launcher.os, 'geteuid', return_value=1000):
            launcher.prepare_host_network(dry_run=True)
        self.assertEqual({path.name: path.read_bytes() for path in self.root.iterdir()}, before)
        self.assertEqual(self.commands, [])
        self.assertIn('dry run; no changes', self.stderr.getvalue())

    def test_bad_or_missing_kernel_value_fails_before_unit_or_systemctl(self):
        for value in ('bad', '0', '-1'):
            self.proc.write_text(value)
            with self.assertRaises(ValueError):
                launcher.prepare_host_network()
        self.proc.unlink()
        with self.assertRaises(FileNotFoundError):
            launcher.prepare_host_network()
        self.assertFalse(self.unit.exists())
        self.assertEqual(self.commands, [])

    def test_unrelated_unit_and_unsafe_unit_permissions_are_preserved_and_rejected(self):
        self.unit.write_text('administrator configuration\n')
        with self.assertRaisesRegex(ValueError, 'differs from the managed'):
            launcher.prepare_host_network()
        self.assertEqual(self.unit.read_text(), 'administrator configuration\n')
        self.unit.write_text(launcher.host_network_unit())
        self.unit.chmod(0o666)
        with self.assertRaisesRegex(ValueError, 'Unsafe host DDS unit'):
            launcher.prepare_host_network()
        self.assertEqual(self.commands, [])
        self.assertEqual(self.proc.read_text(), '212992\n')

    def test_nonroot_owned_boot_code_or_parent_is_rejected_and_apply_needs_root(self):
        self.unit.write_text(launcher.host_network_unit())
        os.chown(self.unit, 1000, -1)
        with self.assertRaises(ValueError):
            launcher.prepare_host_network()
        self.unit.unlink()
        os.chown(self.root, 1000, -1)
        with self.assertRaises(ValueError):
            launcher.prepare_host_network()
        os.chown(self.root, 0, -1)
        with mock.patch.object(launcher.os, 'geteuid', return_value=1000):
            with self.assertRaisesRegex(ValueError, 'with sudo'):
                launcher.prepare_host_network()
        self.assertEqual(self.commands, [])
        self.assertFalse(self.unit.exists())

    def test_symlink_hardlink_and_writable_parent_are_rejected(self):
        target = self.root / 'other.service'
        target.write_text(launcher.host_network_unit())
        self.unit.symlink_to(target)
        with self.assertRaises(ValueError):
            launcher.prepare_host_network()
        self.unit.unlink()
        os.link(target, self.unit)
        with self.assertRaises(ValueError):
            launcher.prepare_host_network()
        self.unit.unlink()
        self.root.chmod(0o777)
        with self.assertRaises(ValueError):
            launcher.prepare_host_network()
        self.assertEqual(self.commands, [])

    def test_failed_atomic_install_leaves_no_partial_unit_or_kernel_change(self):
        with mock.patch.object(launcher.os, 'link', side_effect=OSError('read-only filesystem')):
            with self.assertRaises(OSError):
                launcher.prepare_host_network()
        self.assertFalse(self.unit.exists())
        self.assertFalse(list(self.root.glob('.nuvo-ros-network.*')))
        self.assertEqual(self.commands, [])
        self.assertEqual(self.proc.read_text(), '212992\n')

    def test_interrupted_systemd_setup_is_retried_and_readback_is_required(self):
        self.fail_command = ['systemctl', 'enable']
        with self.assertRaises(subprocess.CalledProcessError):
            launcher.prepare_host_network()
        self.assertTrue(self.unit.exists())
        self.assertEqual(self.proc.read_text(), '212992\n')
        self.fail_command = None
        self.execute_unit = False
        with self.assertRaisesRegex(ValueError, 'still below'):
            launcher.prepare_host_network()
        self.execute_unit = True
        launcher.prepare_host_network()
        self.assertEqual(int(self.proc.read_text()), launcher.DDS_RECEIVE_MINIMUM)

    @unittest.skipUnless(shutil.which('systemd-analyze'), 'systemd unit verifier unavailable')
    def test_unit_passes_systemd_parser(self):
        self.unit.write_text(launcher.host_network_unit())
        result = subprocess.run(['systemd-analyze', 'verify', '--man=no', str(self.unit)],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_main_dry_run_keeps_json_stdout_and_never_mutates_host_or_starts_container(self):
        result, output = self.invoke_main('--dry-run')
        self.assertEqual(result, 0)
        self.assertEqual(json.loads(output)['services']['camera']['image'], 'example:tested')
        self.assertFalse(self.unit.exists())
        self.assertEqual(self.proc.read_text(), '212992\n')
        self.assertFalse(any(command[0] == 'systemctl' or 'up' in command for command in self.commands))

    def test_main_recreates_sockets_on_each_explicit_launch(self):
        self.invoke_main()
        first = next(command for command in self.commands if 'up' in command)
        self.assertIn('--force-recreate', first)
        self.commands.clear()
        self.invoke_main()
        second = next(command for command in self.commands if 'up' in command)
        self.assertIn('--force-recreate', second)

    def test_main_retry_after_failed_compose_recreates_existing_small_buffer_container(self):
        # Allow Compose config, fail only its first up after successful tuning.
        failed = False
        original_run = self.fake_run

        def fail_up(command, **kwargs):
            nonlocal failed
            if 'up' in command and not failed:
                failed = True
                self.commands.append(command)
                raise subprocess.CalledProcessError(1, command)
            return original_run(command, **kwargs)

        with mock.patch.object(launcher, 'run', side_effect=fail_up):
            with self.assertRaises(subprocess.CalledProcessError):
                self.invoke_main()
        self.assertTrue(self.unit.exists())
        self.assertEqual(int(self.proc.read_text()), 8388608)
        self.commands.clear()
        self.invoke_main()
        retry = next(command for command in self.commands if 'up' in command)
        self.assertIn('--force-recreate', retry)

    def test_main_unsafe_unit_and_systemctl_failure_stop_before_compose_up(self):
        self.unit.write_text('unmanaged')
        with self.assertRaises(ValueError):
            self.invoke_main()
        self.assertFalse(any('up' in command for command in self.commands))
        self.unit.unlink()
        self.fail_command = ['systemctl', 'restart']
        with self.assertRaises(subprocess.CalledProcessError):
            self.invoke_main()
        self.assertFalse(any('up' in command for command in self.commands))

    def test_administrator_dropins_are_preserved_and_rejected_before_any_unit_execution(self):
        for directory in (self.unit.parent, *launcher.SYSTEMD_UNIT_DIRS):
            dropins = directory / (self.unit.name + '.d')
            dropins.mkdir(parents=True)
            override = dropins / 'override.conf'
            override.write_text('[Service]\nExecStart=\nExecStart=/bin/false\n')
            self.commands.clear()
            with self.assertRaisesRegex(ValueError, 'administrator drop-ins'):
                self.invoke_main()
            self.assertEqual(override.read_text(), '[Service]\nExecStart=\nExecStart=/bin/false\n')
            self.assertFalse(self.unit.exists())
            self.assertFalse(any(command[0] == 'systemctl' or 'up' in command for command in self.commands))
            self.assertEqual(self.proc.read_text(), '212992\n')
            override.unlink()


if __name__ == '__main__':
    unittest.main()
