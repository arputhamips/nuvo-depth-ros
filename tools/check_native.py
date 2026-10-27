#!/usr/bin/env python3
"""Read-only same-Pi prerequisites. Does not start services or change the camera."""
import argparse
import grp
import json
import os
from pathlib import Path
import stat
import subprocess
import time


def status_errors(status):
    errors = []
    if status.get('status') != 'running':
        errors.append('Native status is not running; inspect the native service journal.')
    if status.get('trigger_hardware_timed') is not True or status.get('trigger_source') != 'rp1-pio-pwm':
        errors.append('Native capture is not using RP1 PIO PWM. See docs/native-startup.md.')
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime-dir', type=Path, default=Path('/run/nuvo-depth'))
    args = parser.parse_args()
    errors = []
    try:
        service = subprocess.run(['systemctl', 'is-active', 'nuvo-depthd-native.service'],
                                 capture_output=True, text=True, timeout=10)
        if service.returncode:
            errors.append('nuvo-depthd-native.service is ' + service.stdout.strip() +
                          '; follow docs/native-startup.md before launching ROS.')
        if not os.access('/usr/lib/nuvo-depth/nuvo-trigger-prepare', os.X_OK):
            errors.append('Native PWM preparation helper is missing. Upgrade the native package cohort; '
                          'ROS cannot repair a legacy GPIO daemon by mounting more devices.')
        gid = grp.getgrnam('nuvo-video').gr_gid
        print(f'NUVO_VIDEO_GID={gid}')
        runtime = args.runtime_dir.stat()
        endpoint = (args.runtime_dir / 'data.sock').stat()
        if not stat.S_ISSOCK(endpoint.st_mode):
            errors.append('data.sock is not a Unix socket.')
        if runtime.st_gid != gid or not runtime.st_mode & stat.S_IXGRP:
            errors.append('Runtime directory must allow traversal by nuvo-video.')
        if endpoint.st_gid != gid or endpoint.st_mode & 0o060 != 0o060:
            errors.append('Socket must grant read/write access to nuvo-video.')
        path = args.runtime_dir / 'status.json'
        status = json.loads(path.read_text())
        if not -1 <= time.time() - path.stat().st_mtime <= 10:
            errors.append('Native status is stale; an old socket alone does not prove capture is running.')
        errors.extend(status_errors(status))
        print('Trigger:', status.get('trigger_source', 'unreported'))
        print('Calibration SHA-256:', status.get('calibration_sha256', 'unreported'))
        print('Calibration authority:', status.get('calibration_authority', 'unreported'))
        if status.get('calibration_authority') not in (1, 2):
            print('PREVIEW ONLY: ROS blocks provisional measurements by default. For an explicit '
                  'inspection, set allow_provisional_calibration: true; this does not validate distances.')
    except (OSError, KeyError, ValueError, subprocess.TimeoutExpired) as error:
        errors.append(str(error) + '; run this check on the Pi host with sudo after native startup.')
    for error in errors:
        print('FAIL:', error)
    if errors:
        return 1
    print('PASS: native startup and socket permissions are ready for the ROS container. '
          'Verify measurement publication with a best-effort subscriber next.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
