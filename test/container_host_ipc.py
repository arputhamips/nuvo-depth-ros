"""Native host socket/clock -> unprivileged Ubuntu ROS container -> host DDS."""
import json
import os
import pathlib
import subprocess
import tempfile
import time

os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
os.environ.setdefault('ROS_DOMAIN_ID', '92')
import rclpy
from integration import Observer, Peer


def associated(observer, run, minimum=1):
    common = set.intersection(*(set(values) for values in observer.messages.values()))
    return sum(observer.messages['metadata'][stamp].run_uuid == run * 32
               for stamp in common) >= minimum


def isolated_acquisition(image, image_id, runtime_uid, group, root, output):
    """Acquisition and ROS publication need no network or shared IPC namespace."""
    name = 'nuvo-isolated-contract-' + str(os.getpid())
    socket_path = root / 'isolated.sock'
    (root / 'isolated.yaml').write_text(
        '/**:\n  ros__parameters:\n    source: local\n'
        '    endpoint: /run/nuvo-depth/isolated.sock\n    lazy: false\n')
    peer = Peer(socket_path, '6')
    socket_path.chmod(0o660)
    process = None
    verified = False
    report = dict(image=image, image_id=image_id, runtime_uid=runtime_uid,
                  network_mode='none', ipc_mode='private',
                  fixture_gid=group, passed=False)
    log_path = output / 'no-network-container.log'
    try:
        with log_path.open('w') as log:
            process = subprocess.Popen(
                ['docker', 'run', '--rm', '--init', '--name', name,
                 '--network', 'none', '--ipc', 'private', '--group-add', str(group),
                 '-e', 'RMW_IMPLEMENTATION=rmw_cyclonedds_cpp',
                 '-v', str(root) + ':/run/nuvo-depth:ro', image,
                 'ros2', 'launch', 'nuvo_bringup', 'camera.launch.py',
                 'namespace:=isolated_contract',
                 'config:=/run/nuvo-depth/isolated.yaml'],
                stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline:
                assert process.poll() is None, 'isolated ROS container exited during acquisition'
                if (peer.requests >= 3 and
                        'First associated depth frame published' in log_path.read_text()):
                    report['publication_observed'] = True
                    break
                time.sleep(.05)
            else:
                raise AssertionError('isolated ROS publication or repeated local acquisition timed out')
            assert peer.connections == 1, 'isolated local source repeatedly reconnected'
            assert peer.subscriptions == [dict(products=40, cadence_millihz=10000,
                                               preview_cadence_millihz=0, reserved=0)], \
                'ROS local demand must contain only depth/validity and no encoded preview'
            verified = True
    except Exception:
        print(log_path.read_text())
        raise
    finally:
        try:
            if process:
                subprocess.run(
                    ['docker', 'stop', '--signal', 'SIGINT', '--time', '4', name],
                    stdout=subprocess.DEVNULL, check=False, timeout=10)
                process.wait(timeout=8)
                report['exit_code'] = process.returncode
                assert process.returncode == 0, 'isolated container did not shut down cleanly'
                deadline = time.monotonic() + 2
                while peer.client is not None and time.monotonic() < deadline:
                    time.sleep(.02)
                assert peer.client is None, 'isolated source retained the host IPC connection'
                report['connection_closed'] = True
                report['passed'] = verified
        finally:
            report.update(connections=peer.connections, frame_requests=peer.requests,
                          subscriptions=peer.subscriptions)
            peer.close()
            (output / 'no-network-contract.json').write_text(json.dumps(report, indent=2) + '\n')
    print('Unprivileged ROS local acquisition/publication passed with no network '
          'and a private IPC namespace')


def main():
    image = os.environ.get('NUVO_TEST_IMAGE', 'nuvo-depth-ros:development')
    # The host process can assign its own group without sudo. The runtime UID
    # differs from the fixture owner and needs this numeric supplementary GID.
    group = int(os.environ.get('NUVO_TEST_GID', os.getgid()))
    name = 'nuvo-contract-' + str(os.getpid())
    rclpy.init()
    try:
        with tempfile.TemporaryDirectory(prefix='nuvo-container-') as directory:
            root = pathlib.Path(directory)
            os.chown(root, os.getuid(), group)
            root.chmod(0o2750)
            output = pathlib.Path(os.environ.get('NUVO_TEST_OUTPUT_DIR', root))
            output.mkdir(parents=True, exist_ok=True)
            (root / 'camera.yaml').write_text(
                '/**:\n  ros__parameters:\n    publish_points: true\n    point_stride: 1\n')
            image_id = subprocess.check_output(
                ['docker', 'image', 'inspect', image, '--format', '{{.Id}}'],
                text=True, timeout=15).strip()
            runtime_uid = int(subprocess.check_output(
                ['docker', 'run', '--rm', image, 'id', '-u'], text=True, timeout=15))
            assert runtime_uid != 0, 'deployment image must run without root'
            assert runtime_uid != os.getuid(), 'fixture owner must differ from runtime UID'
            default_groups = subprocess.check_output(
                ['docker', 'run', '--rm', image, 'id', '-G'],
                text=True, timeout=15).strip()
            assert group not in map(int, default_groups.split()), \
                'fixture GID already belongs to runtime user; set a different NUVO_TEST_GID'
            runtime_groups = subprocess.check_output(
                ['docker', 'run', '--rm', '--group-add', str(group), image, 'id', '-G'],
                text=True, timeout=15).strip()
            assert group in map(int, runtime_groups.split()), 'numeric supplementary GID missing'
            peer = Peer(root / 'data.sock', '2')
            (root / 'data.sock').chmod(0o660)
            observer = Observer('/container_contract')
            process = None
            report = dict(image=image, image_id=image_id, runtime_uid=runtime_uid,
                          runtime_groups=runtime_groups,
                          fixture_gid=group, passed=False)
            with (output / 'container.log').open('w+') as log:
                try:
                    # Prove that access really depends on the explicit GID;
                    # permissive fixture modes or a root image must not pass.
                    denied = subprocess.run(
                        ['docker', 'run', '--rm', '-v', str(root) + ':/run/nuvo-depth:ro',
                         image, 'python3', '-c',
                         'import socket; s=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET); '
                         's.connect("/run/nuvo-depth/data.sock")'],
                        capture_output=True, text=True, timeout=15)
                    assert denied.returncode != 0 and 'PermissionError' in denied.stderr, \
                        'runtime unexpectedly reached fixture without supplementary GID'
                    process = subprocess.Popen(
                        ['docker', 'run', '--rm', '--init', '--name', name,
                         '--network', 'host', '--ipc', 'host', '--group-add', str(group),
                         '-e', 'RMW_IMPLEMENTATION=rmw_cyclonedds_cpp',
                         '-e', 'ROS_DOMAIN_ID=' + os.environ['ROS_DOMAIN_ID'],
                         '-v', str(root) + ':/run/nuvo-depth:ro', image,
                         'ros2', 'launch', 'nuvo_bringup', 'camera.launch.py',
                         'namespace:=container_contract',
                         'config:=/run/nuvo-depth/camera.yaml'],
                        stdout=log, stderr=subprocess.STDOUT)
                    observer.spin(lambda: associated(observer, '2', 3), seconds=20)
                    observer.verify('local_boottime', '2')
                    peer.close()
                    peer = Peer(root / 'data.sock', '4')
                    (root / 'data.sock').chmod(0o660)
                    observer.spin(lambda: associated(observer, '4'), seconds=15)
                    observer.verify('local_boottime', '4')
                    assert peer.subscriptions and all(
                        subscription == dict(products=40, cadence_millihz=10000,
                                             preview_cadence_millihz=0, reserved=0)
                        for subscription in peer.subscriptions), peer.subscriptions
                    report.update(passed=True, run_uuids=['2' * 32, '4' * 32],
                                  subscriptions=peer.subscriptions,
                                  directory_mount_reconnect=True)
                except Exception:
                    log.flush()
                    log.seek(0)
                    print(log.read())
                    raise
                finally:
                    if process:
                        shutdown_started = time.monotonic()
                        subprocess.run(
                            ['docker', 'stop', '--signal', 'SIGINT', '--time', '4', name],
                            stdout=subprocess.DEVNULL, check=False, timeout=10)
                        process.wait(timeout=8)
                        report['shutdown_seconds'] = time.monotonic() - shutdown_started
                        report['exit_code'] = process.returncode
                        if process.returncode != 0:
                            report['passed'] = False
                    peer.close()
                    observer.node.destroy_node()
                    (output / 'contract.json').write_text(json.dumps(report, indent=2) + '\n')
                    if process:
                        assert process.returncode == 0, 'container did not shut down cleanly'
            isolated_acquisition(image, image_id, runtime_uid, group, root, output)
    finally:
        rclpy.shutdown()
    print('Native host IPC, clock correlation, unprivileged UID, required numeric GID, '
          'read-only directory mount and container DDS reconnect passed')


if __name__ == '__main__':
    main()
