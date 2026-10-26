"""728x544 DDS images, disparity, XYZ, association and subscriber-demand test.

Host: source install/setup.bash; python3 test/fullsize.py
Container: NUVO_TEST_IMAGE=nuvo-depth-ros:ci python3 test/fullsize.py --container
This is synthetic host/container evidence, not Pi hardware qualification.
"""
import argparse
import json
import math
import os
from pathlib import Path
import struct
import subprocess
import tempfile
import time

os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
os.environ.setdefault('ROS_DOMAIN_ID', '92')
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from stereo_msgs.msg import DisparityImage
from nuvo_camera_msgs.msg import FrameMetadata
from integration import Peer, fixture, packet, stop

W, H = 728, 544
COUNT = W * H
VALID = bytes(6 if i % W < 64 or i == COUNT - 2 else 1 for i in range(COUNT))
DEPTH = b''.join(struct.pack('<f', 2. if code == 1 else math.nan) for code in VALID)
DISPARITY = b''.join(struct.pack('<f', 23. if code == 1 else math.nan) for code in VALID)
LEFT = bytes(i % 251 for i in range(COUNT))
RIGHT = bytes(i % 253 for i in range(COUNT * 3))
PAYLOAD = DEPTH + VALID + DISPARITY + LEFT + RIGHT


def ip_reassembly():
    lines = Path('/proc/net/snmp').read_text().splitlines()
    keys, values = (line.split()[1:] for line in lines[:2])
    counters = dict(zip(keys, map(int, values)))
    return {key: counters[key] for key in ('ReasmReqds', 'ReasmOKs', 'ReasmFails')}


def fullsize_packet(sequence, run):
    wire = packet(sequence, run, live=True)
    size = fixture.HEADER.unpack_from(wire)[5]
    document = json.loads(wire[32:32 + size])
    grid = dict(width=W, height=H, reference_width=W, reference_height=H,
                x_scale_to_reference=1., y_scale_to_reference=1.,
                x_offset_to_reference=0., y_offset_to_reference=0.)
    document['image_grid'] = document['matching_grid'] = grid
    left = [500., 0., 364., 0., 0., 500., 272., 0., 0., 0., 1., 0.]
    right = left.copy()
    right[2], right[3] = 366., -50.
    geometry = dict(left_projection=left, right_projection=right,
                    reprojection=[1., 0., 0., -364., 0., 1., 0., -272.,
                                  0., 0., 0., 500., 0., 0., 10., 20.], baseline_m=.1)
    document['calibration_geometry'] = dict(image=geometry, matching=geometry)
    document['depth_metadata'] = dict(minimum_m=.1, maximum_m=20., invalid_is_nan=True)
    document['disparity_metadata'] = dict(
        valid_roi=dict(x=64, y=0, width=W - 64, height=H),
        unit='matching_grid_pixels', quantization_px=.0625,
        accepted_min_px=0., accepted_max_px=64., invalid_is_nan=True,
        confidence_model='binary_sgbm_support_v1', derivation='native_matcher_grid')
    products = []
    offset = 0
    for kind, format_, data in (
            ('depth', 'float32_le', DEPTH), ('validity', 'uint8', VALID),
            ('disparity', 'float32_le', DISPARITY),
            ('left_rectified', 'gray8', LEFT), ('right_rectified', 'rgb8', RIGHT)):
        products.append(dict(kind=kind, format=format_, width=W, height=H,
                             offset=offset, length=len(data)))
        offset += len(data)
    document.update(products=products, selected_products=47, available_products=47)
    return fixture.encode(document, PAYLOAD, sequence)


class FullObserver:
    TYPES = {
        'depth': ('depth/image_rect', Image), 'validity': ('depth/validity', Image),
        'info': ('depth/camera_info', CameraInfo), 'points': ('points', PointCloud2),
        'metadata': ('frame_metadata', FrameMetadata),
        'left': ('left/image_rect', Image), 'left_info': ('left/camera_info', CameraInfo),
        'right': ('right/image_rect', Image), 'right_info': ('right/camera_info', CameraInfo),
        'disparity': ('disparity', DisparityImage)}

    def __init__(self):
        self.node = rclpy.create_node('nuvo_fullsize_observer')
        self.messages = {key: {} for key in self.TYPES}
        self.subscriptions = {}
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        for key, (topic, kind) in self.TYPES.items():
            self.subscriptions[key] = self.node.create_subscription(
                kind, '/fullsize_contract/' + topic,
                lambda message, key=key: self.record(key, message), qos)

    def record(self, key, message):
        stamp = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
        values = self.messages[key]
        values[stamp] = message
        while len(values) > 4:
            del values[min(values)]

    def coherent(self):
        return set.intersection(*(set(messages) for messages in self.messages.values()))

    def wait(self, predicate, seconds=20):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=.1)
            if predicate():
                return
        raise AssertionError('full-size DDS timeout: ' + str({k: len(v) for k, v in self.messages.items()}))

    def verify(self):
        stamp = max(self.coherent())
        m = {key: values[stamp] for key, values in self.messages.items()}
        for key in ('depth', 'validity', 'left', 'right'):
            assert (m[key].width, m[key].height) == (W, H), key
        assert (m['depth'].step, len(m['depth'].data)) == (W * 4, COUNT * 4)
        assert bytes(m['validity'].data) == VALID
        assert bytes(m['left'].data) == LEFT and m['left'].encoding == 'mono8'
        assert bytes(m['right'].data) == RIGHT and m['right'].encoding == 'rgb8'
        assert m['right'].step == W * 3
        assert m['right'].header.frame_id == m['right_info'].header.frame_id
        assert m['right'].header.frame_id != m['left'].header.frame_id
        for key in ('info', 'left_info', 'right_info'):
            assert (m[key].width, m[key].height) == (W, H)
        assert m['left_info'].p[0] == 500. and m['left_info'].p[2] == 364.
        assert m['right_info'].p[2] == 366. and m['right_info'].p[3] == -50.
        d = m['disparity']
        assert d.header == d.image.header == m['depth'].header
        assert (d.image.width, d.image.height, d.image.step) == (W, H, W * 4)
        assert (d.f, d.t, d.min_disparity, d.max_disparity, d.delta_d) == (500., struct.unpack('<f', struct.pack('<f', .1))[0], 2., 66., .0625)
        assert (d.valid_window.x_offset, d.valid_window.y_offset,
                d.valid_window.width, d.valid_window.height) == (64, 0, W - 64, H)
        points = m['points']
        assert (points.width, points.height, points.point_step, points.row_step) == (W, H, 12, W * 12)
        assert len(points.data) == COUNT * 12 and not points.is_dense
        depth, disparity, xyz = bytes(m['depth'].data), bytes(d.image.data), bytes(points.data)
        for index in (0, 63, 64, W + 64, COUNT // 2, COUNT - 2, COUNT - 1):
            z = struct.unpack_from('<f', depth, index * 4)[0]
            dis = struct.unpack_from('<f', disparity, index * 4)[0]
            point = struct.unpack_from('<fff', xyz, index * 12)
            if VALID[index] != 1:
                assert math.isnan(z) and math.isnan(dis) and all(math.isnan(a) for a in point)
            else:
                assert z == 2. and dis == 25.
                expected = ((index % W - 364.) * 2. / 500.,
                            (index // W - 272.) * 2. / 500., 2.)
                assert all(math.isclose(a, b, abs_tol=1e-6) for a, b in zip(point, expected))
        assert m['metadata'].run_uuid == '7' * 32
        assert m['metadata'].calibration_sha256 == '3' * 64
        return stamp


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--container', action='store_true')
    args = parser.parse_args()
    output = Path(os.environ.get('NUVO_TEST_OUTPUT_DIR', 'output/fullsize'))
    output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    with tempfile.TemporaryDirectory(prefix='nuvo-fullsize-') as directory:
        root = Path(directory)
        root.chmod(0o750)
        peer = Peer(root / 'data.sock', '7', fullsize_packet)
        (root / 'data.sock').chmod(0o660)
        name = 'nuvo-fullsize-contract-' + str(os.getpid())
        parameters = dict(source='local', endpoint=str(root / 'data.sock'),
                          publish_points=True, point_stride=1, publish_left_rectified=True,
                          publish_right_rectified=True, publish_disparity=True)
        if args.container:
            image = os.environ.get('NUVO_TEST_IMAGE', 'nuvo-depth-ros:ci')
            parameters['endpoint'] = '/run/nuvo-depth/data.sock'
            config = '/**:\n  ros__parameters:\n' + ''.join('    ' + key + ': ' + (str(value).lower() if isinstance(value, bool) else str(value)) + '\n' for key, value in parameters.items())
            (root / 'camera.yaml').write_text(config)
            command = ['docker', 'run', '--rm', '--init', '--name', name,
                       '--network', 'host', '--ipc', 'host', '--group-add', str(os.getgid()),
                       '-e', 'RMW_IMPLEMENTATION=' + os.environ.get('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
                       '-e', 'ROS_DOMAIN_ID=' + os.environ.get('ROS_DOMAIN_ID', '92'),
                       '-v', str(root) + ':/run/nuvo-depth:ro', image,
                       'ros2', 'launch', 'nuvo_bringup', 'camera.launch.py',
                       'namespace:=fullsize_contract', 'config:=/run/nuvo-depth/camera.yaml']
        else:
            prefix = subprocess.check_output(['ros2', 'pkg', 'prefix', 'nuvo_camera'], text=True).strip()
            command = [prefix + '/lib/nuvo_camera/nuvo_camera_node', '--ros-args', '-r', '__ns:=/fullsize_contract']
            for key, value in parameters.items():
                command += ['-p', key + ':=' + (str(value).lower() if isinstance(value, bool) else str(value))]
        report = dict(grid=[W, H], container=args.container, passed=False,
                      rmw=os.environ.get('RMW_IMPLEMENTATION'),
                      cyclonedds_uri=os.environ.get('CYCLONEDDS_URI'),
                      fastdds_profile=os.environ.get('FASTDDS_DEFAULT_PROFILES_FILE'),
                      cloud_bytes=COUNT * 12, depth_bytes=COUNT * 4,
                      host_kernel_limits={name: Path('/proc/sys/net/' + path).read_text().strip()
                          for name, path in [('rmem_max', 'core/rmem_max'), ('wmem_max', 'core/wmem_max'),
                                             ('ipfrag_high_thresh', 'ipv4/ipfrag_high_thresh')]},
                      ip_reassembly_before=ip_reassembly())
        observer = None
        with (output / ('fullsize-container.log' if args.container else 'fullsize-host.log')).open('w+') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                time.sleep(.8)
                assert process.poll() is None and peer.connections == 0, 'lazy full-size source connected without subscribers'
                observer = FullObserver()
                observer.wait(lambda: len(observer.coherent()) >= 2)
                report['associated_stamp_ns'] = observer.verify()
                assert peer.subscriptions[-1]['products'] == 47, peer.subscriptions
                assert all(s['preview_cadence_millihz'] == 0 for s in peer.subscriptions)
                for key in ('points', 'left', 'left_info', 'right', 'right_info', 'disparity'):
                    observer.node.destroy_subscription(observer.subscriptions.pop(key))
                observer.wait(lambda: peer.subscriptions[-1]['products'] == 40)
                for subscription in observer.subscriptions.values():
                    observer.node.destroy_subscription(subscription)
                observer.subscriptions.clear()
                observer.wait(lambda: peer.client is None)
                requests = peer.requests
                time.sleep(.4)
                assert peer.requests == requests, 'full-size source retained demand without subscribers'
                report.update(passed=True, subscriptions=peer.subscriptions, frame_requests=requests,
                              optional_demand_released=True, all_demand_released=True)
            except Exception as error:
                report['failure'] = str(error)
                log.flush(); log.seek(0); print(log.read())
                raise
            finally:
                if args.container:
                    subprocess.run(['docker', 'stop', '--signal', 'SIGINT', '--time', '4', name], check=False, capture_output=True, timeout=10)
                    process.wait(timeout=8)
                    assert process.returncode == 0, process.returncode
                else:
                    stop(process)
                peer.close()
                if observer:
                    observer.node.destroy_node()
                report['ip_reassembly_after'] = ip_reassembly()
                (output / ('fullsize-container.json' if args.container else 'fullsize-host.json')).write_text(json.dumps(report, indent=2) + '\n')
    rclpy.shutdown()
    print('728x544 DDS depth/validity/images/disparity/XYZ association and subscriber-demand checks passed')


if __name__ == '__main__':
    main()
