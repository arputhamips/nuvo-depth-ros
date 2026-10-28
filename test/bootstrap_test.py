"""Run the public bootstrap offline with a fake registry/archive and launcher."""
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'install-nuvo-depth-ros.sh'
PACKAGES = ('nuvo_camera', 'nuvo_camera_msgs', 'nuvo_bringup', 'nuvo_depth_client_vendor')
VERSION = '0.1.2'
REVISION = 'a' * 40
REPOSITORY = 'ghcr.io/arputhamips/nuvo-depth-ros'
DIGEST = REPOSITORY + '@sha256:' + 'b' * 64


class ReleaseMetadata(unittest.TestCase):
    def test_bootstrap_packages_and_deployment_share_a_release(self):
        self.assertIn(f'VERSION={VERSION}\n', SCRIPT.read_text())
        self.assertIn(f'ARG DRIVER_VERSION={VERSION}\n', (ROOT / 'docker/Dockerfile').read_text())
        for package in PACKAGES:
            self.assertEqual(ET.parse(ROOT / package / 'package.xml').findtext('version'), VERSION)
        for compose in ('compose.yaml', 'compose.remote.yaml'):
            self.assertIn(f'{REPOSITORY}:jazzy-{VERSION}', (ROOT / 'docker' / compose).read_text())
        defaults = (ROOT / 'nuvo_bringup/config/camera.yaml').read_text()
        self.assertRegex(defaults, r'(?m)^    allow_provisional_calibration: false$')
        self.assertRegex(defaults, r'(?m)^    calibration_file: ""$')


@unittest.skipUnless(sys.platform == 'linux' and os.geteuid() == 0 and shutil.which('bash'),
                     'Bootstrap runs as root on Linux (CI container or sudo python3)')
class Bootstrap(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='nuvo bootstrap ')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.bin = self.root / 'bin'
        self.bin.mkdir()
        self.calls = self.root / 'calls.jsonl'
        self.arguments = self.root / 'arguments.json'
        self.metadata = self.root / 'image.json'
        self.metadata.write_text(json.dumps([{'Config': {'Labels': {
            'org.opencontainers.image.version': VERSION,
            'org.opencontainers.image.revision': REVISION}}, 'RepoDigests': [DIGEST]}]))
        self.archive = self.root / 'source.tar.gz'
        self.make_archive()
        self.make_command('docker', '''
import json, os, sys
from pathlib import Path
with open(os.environ['FAKE_CALLS'], 'a') as stream:
    stream.write(json.dumps(sys.argv[1:]) + '\\n')
if sys.argv[1:] == ['compose', 'version'] and os.environ.get('FAIL_COMPOSE'):
    sys.exit(1)
if sys.argv[1:3] == ['image', 'inspect']:
    print(Path(os.environ['FAKE_METADATA']).read_text())
''')
        self.make_command('curl', '''
import json, os, shutil, sys
with open(os.environ['FAKE_CALLS'], 'a') as stream:
    stream.write(json.dumps(['curl'] + sys.argv[1:]) + '\\n')
if os.environ.get('FAIL_DOWNLOAD'):
    sys.exit(22)
assert 'https://codeload.github.com/arputhamips/nuvo-depth-ros/tar.gz/' + 'a' * 40 in sys.argv
shutil.copyfile(os.environ['FAKE_ARCHIVE'], sys.argv[sys.argv.index('--output') + 1])
''')
        self.env = dict(os.environ, PATH=str(self.bin) + os.pathsep + os.environ['PATH'],
                        FAKE_CALLS=str(self.calls), FAKE_METADATA=str(self.metadata),
                        FAKE_ARCHIVE=str(self.archive), FAKE_ARGUMENTS=str(self.arguments))

    def make_command(self, name, code):
        path = self.bin / name
        path.write_text('#!' + sys.executable + '\n' + code)
        path.chmod(0o755)

    def make_archive(self, version=VERSION):
        files = {f'{package}/package.xml': f'<package><version>{version}</version></package>'
                 for package in PACKAGES}
        files.update({
            'tools/run_camera.py': 'import json, os, sys\nfrom pathlib import Path\n'
                                   'Path(os.environ["FAKE_ARGUMENTS"]).write_text(json.dumps(sys.argv[1:]))\n',
            'tools/check_native.py': '# read-only preflight\n',
            'docker/compose.yaml': f'services: {{camera: {{image: {REPOSITORY}:jazzy-{VERSION}}}}}\n',
            'nuvo_bringup/config/camera.yaml': (ROOT / 'nuvo_bringup/config/camera.yaml').read_text()})
        with tarfile.open(self.archive, 'w:gz') as archive:
            for name, content in files.items():
                data = content.encode()
                info = tarfile.TarInfo('nuvo-depth-ros-' + REVISION + '/' + name)
                info.size = len(data)
                archive.addfile(info, io.BytesIO(data))

    def run_script(self, *args, piped=False, success=True):
        command = ['bash', '-s', '--'] if piped else ['bash', str(SCRIPT)]
        command += ['--install-dir', str(self.root / 'install'), '--state-dir', str(self.root / 'state'), *args]
        result = subprocess.run(command, input=SCRIPT.read_text() if piped else None,
                                env=self.env, text=True, capture_output=True, timeout=15)
        if success:
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        else:
            self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertFalse(self.arguments.exists())
        return result

    def test_piped_first_install_pins_image_and_source_and_preserves_safe_defaults(self):
        self.run_script(piped=True)
        arguments = json.loads(self.arguments.read_text())
        self.assertEqual(arguments, ['--image', DIGEST, '--config', str(self.root / 'state/camera.yaml')])
        self.assertEqual((self.root / 'state/camera.yaml').read_bytes(),
                         (ROOT / 'nuvo_bringup/config/camera.yaml').read_bytes())
        self.assertEqual([path.name for path in (self.root / 'state').iterdir()], ['camera.yaml'])
        self.assertEqual((self.root / f'install/releases/{REVISION}/.nuvo-source-revision').read_text().strip(), REVISION)
        calls = [json.loads(line) for line in self.calls.read_text().splitlines()]
        self.assertIn(['pull', REPOSITORY + ':jazzy-' + VERSION], calls)
        self.assertEqual(sum(call[0] == 'curl' for call in calls), 1)

    def test_saved_script_rerun_retains_user_config_and_reuses_matching_source(self):
        self.run_script()
        config = self.root / 'state/camera.yaml'
        config.write_text('user settings must survive\n')
        self.env['FAIL_DOWNLOAD'] = '1'
        self.run_script('--outputs', 'depth,points', '--cadence-hz', '7.5', '--namespace', 'robot/front',
                        '--ros-domain-id', '42', '--calibration-file', '/home/pi/my calibration.tar',
                        '--allow-provisional-calibration', '--dry-run')
        self.assertEqual(config.read_text(), 'user settings must survive\n')
        self.assertEqual(json.loads(self.arguments.read_text())[4:], [
            '--outputs', 'depth,points', '--cadence-hz', '7.5', '--namespace', 'robot/front',
            '--ros-domain-id', '42', '--calibration-file', '/home/pi/my calibration.tar',
            '--allow-provisional-calibration', '--dry-run'])

    def test_explicit_config_is_forwarded_without_creating_default(self):
        config = self.root / 'my parameters.yaml'
        config.write_text('unchanged\n')
        self.run_script('--config', str(config), '--outputs', 'left')
        self.assertEqual(config.read_text(), 'unchanged\n')
        self.assertFalse((self.root / 'state').exists())
        self.assertEqual(json.loads(self.arguments.read_text()), ['--image', DIGEST, '--config', str(config), '--outputs', 'left'])

    def test_wrong_registry_metadata_cannot_execute_downloaded_code(self):
        original = self.metadata.read_text()
        for change in ('version', 'revision', 'digest'):
            with self.subTest(change=change):
                metadata = json.loads(original)
                if change == 'digest':
                    metadata[0]['RepoDigests'] = ['other/repository@sha256:' + 'b' * 64]
                else:
                    metadata[0]['Config']['Labels']['org.opencontainers.image.' + change] = 'wrong'
                self.metadata.write_text(json.dumps(metadata))
                self.run_script(success=False)
                self.assertFalse((self.root / 'install').exists())

    def test_wrong_source_version_and_failed_download_do_not_install_or_start(self):
        self.make_archive(version='0.1.1')
        self.run_script(success=False)
        self.assertFalse((self.root / f'install/releases/{REVISION}').exists())
        self.assertFalse((self.root / 'state').exists())
        self.env['FAIL_DOWNLOAD'] = '1'
        self.run_script(success=False)
        self.assertEqual(list((self.root / 'install/releases').iterdir()), [])

    def test_prerequisite_error_is_actionable_and_does_not_download(self):
        self.env['FAIL_COMPOSE'] = '1'
        result = self.run_script(success=False)
        self.assertIn('Docker Compose plugin is required', result.stderr)
        self.assertFalse((self.root / 'install').exists())

    def test_unknown_options_fail_before_docker_or_download(self):
        self.run_script('--image', 'some-other:image', success=False)
        self.assertFalse(self.calls.exists())

    def test_existing_empty_parameters_are_preserved(self):
        state = self.root / 'state'
        state.mkdir()
        (state / 'camera.yaml').touch()
        self.run_script()
        self.assertEqual((state / 'camera.yaml').read_bytes(), b'')

    def test_reuse_rejects_writable_cached_code(self):
        self.run_script()
        self.arguments.unlink()
        (self.root / f'install/releases/{REVISION}/tools/run_camera.py').chmod(0o666)
        result = self.run_script(success=False)
        self.assertIn('Refusing unsafe cached deployment', result.stderr)

    def test_cache_cannot_traverse_user_writable_directory_or_symlink(self):
        unsafe = self.root / 'unsafe'
        unsafe.mkdir(mode=0o777)
        unsafe.chmod(0o777)
        self.run_script('--install-dir', str(unsafe / 'cache'), success=False)
        link = self.root / 'redirect'
        link.symlink_to(unsafe, target_is_directory=True)
        self.run_script('--install-dir', str(link / 'cache'), success=False)
        self.assertFalse((unsafe / 'cache').exists())


if __name__ == '__main__':
    unittest.main()
