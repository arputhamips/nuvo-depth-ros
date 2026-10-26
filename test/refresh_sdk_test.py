"""Pinned SDK checks detect source and provenance drift, including with -O."""
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'tools/refresh_sdk.py'
spec = importlib.util.spec_from_file_location('refresh_sdk', SCRIPT)
refresh = importlib.util.module_from_spec(spec)
spec.loader.exec_module(refresh)


class SnapshotTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.destination = self.root / 'nuvo_depth_client_vendor/upstream'
        self.destination.mkdir(parents=True)
        (self.destination / 'client.cpp').write_bytes(b'original\n')
        self.files = {'client.cpp': {
            'upstream_path': 'sdk/cpp/src/client.cpp',
            'sha256': hashlib.sha256(b'original\n').hexdigest(),
        }}
        self.manifest = {'schema_version': 1, 'files': self.files}
        self.save_manifest()

    def save_manifest(self):
        self.manifest['content_sha256'] = hashlib.sha256(
            json.dumps(self.files, sort_keys=True, separators=(',', ':')).encode()
        ).hexdigest()
        (self.destination / 'SOURCE-MANIFEST.json').write_text(json.dumps(self.manifest))

    def test_valid_snapshot(self):
        self.assertEqual(refresh.check_snapshot(self.destination), self.manifest)

    def test_source_drift_rejected_in_normal_and_optimized_python(self):
        (self.root / 'tools').mkdir()
        shutil.copyfile(SCRIPT, self.root / 'tools/refresh_sdk.py')
        (self.destination / 'client.cpp').write_bytes(b'changed\n')
        for flags in ([], ['-O']):
            result = subprocess.run(
                [sys.executable, *flags, str(self.root / 'tools/refresh_sdk.py'), '--check'],
                capture_output=True, text=True,
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('checksum mismatch', result.stderr)

    def test_stale_aggregate_provenance_rejected(self):
        self.files['client.cpp']['upstream_path'] = 'wrong/source.cpp'
        (self.destination / 'SOURCE-MANIFEST.json').write_text(json.dumps(self.manifest))
        with self.assertRaisesRegex(ValueError, 'content_sha256'):
            refresh.check_snapshot(self.destination)

    def test_unexpected_file_rejected(self):
        (self.destination / 'obsolete.cpp').write_text('obsolete')
        with self.assertRaisesRegex(ValueError, 'unexpected'):
            refresh.check_snapshot(self.destination)

    def test_path_escape_rejected(self):
        self.files['../../outside.cpp'] = self.files.pop('client.cpp')
        self.save_manifest()
        with self.assertRaisesRegex(ValueError, 'invalid SDK manifest path'):
            refresh.check_snapshot(self.destination)

    def test_unknown_schema_rejected(self):
        self.manifest['schema_version'] = 2
        self.save_manifest()
        with self.assertRaisesRegex(ValueError, 'unsupported'):
            refresh.check_snapshot(self.destination)


if __name__ == '__main__':
    unittest.main()
