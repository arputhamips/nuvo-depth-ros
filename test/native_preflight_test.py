"""Old, stopped and userspace-trigger status must never pass Pi startup checks."""
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('check_native', Path(__file__).resolve().parents[1] / 'tools/check_native.py')
check_native = importlib.util.module_from_spec(spec)
spec.loader.exec_module(check_native)


class NativeStatus(unittest.TestCase):
    def test_requires_running_hardware_timed_capture(self):
        valid = dict(status='running', trigger_source='rp1-pio-pwm', trigger_hardware_timed=True)
        self.assertEqual(check_native.status_errors(valid), [])
        for update in ({'status': 'stopped'}, {'trigger_hardware_timed': False},
                       {'trigger_source': 'userspace-gpio'}):
            self.assertTrue(check_native.status_errors(dict(valid, **update)))
        self.assertTrue(check_native.status_errors({}))


if __name__ == '__main__':
    unittest.main()
