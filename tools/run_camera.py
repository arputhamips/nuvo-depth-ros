#!/usr/bin/env python3
"""Start the same-device ROS container with native permissions discovered locally."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import stat
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
OUTPUTS = {'depth': 'publish_depth', 'points': 'publish_points',
           'left': 'publish_left_rectified', 'right': 'publish_right_rectified',
           'disparity': 'publish_disparity'}
CONTROLS_LABEL = 'org.nuvo.ros.launch-controls'
BUNDLE_FILES = ('manifest.json', 'front_left.yaml', 'front_right.yaml', 'validation.json')
INPUT_CACHE = Path('/var/cache/nuvo-depth-ros/inputs')


def output_selection(value):
    selected = value.split(',')
    if not selected or any(item not in OUTPUTS for item in selected):
        raise argparse.ArgumentTypeError('Choose comma-separated depth,points,left,right,disparity.')
    return set(selected)


def cadence(value):
    result = float(value)
    if not math.isfinite(result) or not 0.001 <= result <= 60:
        raise argparse.ArgumentTypeError('cadence-hz must be finite and between 0.001 and 60.')
    return result


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument('--image', help='Override the Compose image, e.g. a tested development image.')
    result.add_argument('--config', type=Path, help='Optional ROS YAML file; mounted read-only.')
    result.add_argument('--namespace', help='ROS topic namespace (default follows Compose/.env).')
    result.add_argument('--ros-domain-id', type=int, choices=range(233), metavar='0..232')
    result.add_argument('--outputs', type=output_selection,
                        help='Comma-separated depth,points,left,right,disparity; omitted keeps YAML.')
    result.add_argument('--cadence-hz', type=cadence, help='Requested delivery ceiling, not guaranteed capture FPS.')
    result.add_argument('--allow-provisional-calibration', action='store_true', default=None,
                        help='Explicitly permit unqualified preview; preserves calibration authority.')
    result.add_argument('--calibration-file', type=Path,
                        help='Explicit native stereo bundle directory or .tar; changes the camera selection.')
    result.add_argument('--dry-run', action='store_true', help='Inspect native readiness and print resolved deployment only.')
    return result


def run(command, **kwargs):
    return subprocess.run(command, check=True, text=True, **kwargs)


def overrides(args):
    values = {}
    if args.outputs is not None:
        values.update({parameter: str(name in args.outputs).lower() for name, parameter in OUTPUTS.items()})
    if args.cadence_hz is not None:
        values['cadence_hz'] = str(args.cadence_hz)
    if args.allow_provisional_calibration is not None:
        values['allow_provisional_calibration'] = 'true'
    if args.calibration_file is not None:
        values['calibration_file'] = '/calibration/bundle'
    return values


def readonly_mount(path, target):
    path = path.expanduser().resolve(strict=True)
    if not (path.is_file() or path.is_dir()):
        raise ValueError(f'Not a regular file or directory: {path}')
    return {'type': 'bind', 'source': str(path), 'target': target, 'read_only': True,
            'bind': {'create_host_path': False}}


def snapshot_input(source, destination, *, bundle=False, gid=None):
    """Keep a bounded, readable snapshot for an unprivileged container restart.

    Original uploads may be mode 0600. Copy data, never keys, links or modes;
    do not chmod the user's calibration or bind-mount its parent directory.
    """
    source = source.expanduser().absolute()
    directory = bundle and source.is_dir() and not source.is_symlink()
    files = [(name, source / name) for name in BUNDLE_FILES] if directory else [('input', source)]
    limit = 4 * 1024 * 1024 if directory else (17 * 1024 * 1024 if bundle else 1024 * 1024)
    contents = {}
    digest = hashlib.sha256()
    for name, path in files:
        if not stat.S_ISREG(path.lstat().st_mode):
            raise ValueError(f'Expected a regular input file without links: {path}')
        with path.open('rb') as stream:
            data = stream.read(limit + 1)
        if len(data) > limit:
            raise ValueError(f'Input file exceeds size limit: {path}')
        contents[name] = data
        digest.update(name.encode() + b'\0' + len(data).to_bytes(8, 'big') + data)
    destination = destination.absolute()
    if destination.resolve() != destination:
        raise ValueError('Input snapshot cache must not traverse symbolic links')
    destination.mkdir(parents=True, exist_ok=True)
    if destination.is_symlink():
        raise ValueError('Input snapshot cache must not be a symbolic link')
    if sys.platform == 'linux' and (destination.stat().st_uid != os.geteuid() or destination.stat().st_mode & 0o022):
        raise ValueError('Input snapshot cache must be owned by the launcher user and not writable by others')
    target = destination / digest.hexdigest()
    target.mkdir(parents=True, exist_ok=True)
    if target.is_symlink() or not target.is_dir():
        raise ValueError('Input snapshot directory must not be a symbolic link')
    if sys.platform == 'linux' and (target.stat().st_uid != os.geteuid() or target.stat().st_mode & 0o022):
        raise ValueError('Input snapshot directory must be owned by the launcher user and not writable by others')
    target.chmod(0o750)
    if gid is not None:
        os.chown(target, -1, gid)
    for name, data in contents.items():
        path = target / name
        if path.exists():
            if path.is_symlink() or path.stat().st_nlink != 1 or path.read_bytes() != data:
                raise ValueError(f'Existing input snapshot is inconsistent: {path}')
        else:
            with path.open('xb') as stream:
                stream.write(data)
        path.chmod(0o440)
        if gid is not None:
            os.chown(path, -1, gid)
    return target if directory else target / 'input'


def configure(document, args):
    """Apply explicit options only, retaining the resolved Compose environment."""
    service = document['services']['camera']
    command = list(service['command'])
    values = overrides(args)
    if args.namespace is not None:
        values['namespace'] = args.namespace
    if args.config is not None:
        if not args.config.expanduser().is_file():
            raise ValueError('--config must name a readable ROS YAML file')
        service.setdefault('volumes', []).append(readonly_mount(args.config, '/nuvo-parameters.yaml'))
        values['config'] = '/nuvo-parameters.yaml'
    if args.calibration_file is not None:
        service.setdefault('volumes', []).append(readonly_mount(args.calibration_file, '/calibration/bundle'))
    command = [item for item in command if item.split(':=', 1)[0] not in values]
    service['command'] = command + [f'{key}:={value}' for key, value in values.items()]
    if args.ros_domain_id is not None:
        service.setdefault('environment', {})['ROS_DOMAIN_ID'] = str(args.ros_domain_id)
    return document


def main(argv=None):
    args = parser().parse_args(argv)
    if sys.platform != 'linux':
        raise ValueError('Run this helper on the Linux camera host, locally or over SSH.')
    import grp
    gid = grp.getgrnam('nuvo-video').gr_gid
    # A requested bundle can enable previously deferred capture through native
    # validation. Otherwise check service health, fresh status, PIO and access.
    if args.calibration_file is None:
        run([sys.executable, str(ROOT / 'tools/check_native.py')])
    else:
        runtime = Path('/run/nuvo-depth').stat()
        control = Path('/run/nuvo-depth/calibration.sock').stat()
        if (runtime.st_gid != gid or not runtime.st_mode & stat.S_IXGRP or
                not stat.S_ISSOCK(control.st_mode) or control.st_gid != gid or control.st_mode & 0o060 != 0o060):
            raise ValueError('Native calibration socket is not ready; install a compatible native package.')
    environment = dict(os.environ, NUVO_VIDEO_GID=str(gid))
    if args.image:
        environment['NUVO_IMAGE'] = args.image
    compose = ['docker', 'compose', '--project-directory', str(ROOT / 'docker')]
    if (ROOT / 'docker/.env').is_file():
        compose += ['--env-file', str(ROOT / 'docker/.env')]
    base = compose + ['-f', str(ROOT / 'docker/compose.yaml')]
    document = json.loads(run(base + ['config', '--format', 'json'], env=environment, capture_output=True).stdout)
    if args.dry_run:
        configure(document, args)
        print(json.dumps(document, indent=2))
        return 0
    image = document['services']['camera']['image']
    inspect = ['docker', 'image', 'inspect', image]
    result = subprocess.run(inspect, capture_output=True, text=True)
    if result.returncode:
        run(['docker', 'pull', image])
        result = run(inspect, capture_output=True)
    labels = json.loads(result.stdout)[0]['Config'].get('Labels') or {}
    if labels.get(CONTROLS_LABEL) != '1':
        raise ValueError('This image lacks the helper startup controls. Use a compatible newer image '
                         'or --image with a build from this checkout; v0.1.1 does not include them.')
    for option in ('config', 'calibration_file'):
        source = getattr(args, option)
        if source is not None:
            setattr(args, option, snapshot_input(source, INPUT_CACHE,
                                                 bundle=option == 'calibration_file', gid=gid))
    configure(document, args)
    # Compose already resolved relative paths against its original directory.
    # A temporary JSON override avoids editing .env, YAML or user calibration.
    with tempfile.TemporaryDirectory(prefix='nuvo-ros-') as directory:
        path = Path(directory) / 'compose.json'
        path.write_text(json.dumps(document))
        run(compose + ['-f', str(path), 'up', '-d', '--no-build'], env=environment)
    print('Started. Logs: sudo env ' + shlex.quote(f'NUVO_VIDEO_GID={gid}') + ' ' +
          shlex.join(base + ['logs', '--tail=50', 'camera']))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, KeyError, ValueError, subprocess.CalledProcessError) as error:
        print(f'Cannot start Nuvo ROS: {error}', file=sys.stderr)
        raise SystemExit(1)
