"""Check unchanged bytes emitted by native depthd_frame_wire_test over ROS DDS.

Usage: python3 test/native_recording.py /path/to/native/geometry.nuv
The native executable writes that fixture when given an existing output folder.
"""
import json
import math
import pathlib
import struct
import subprocess
import sys
import tempfile

import rclpy

from integration import Observer, fixture, start, stop, write_recording


def main():
    wire = pathlib.Path(sys.argv[1]).read_bytes()
    metadata_size = fixture.HEADER.unpack_from(wire)[5]
    document = json.loads(wire[32:32 + metadata_size])
    payload = wire[32 + metadata_size:]
    products = {product['kind']: product for product in document['products']}
    grid = document['matching_grid']
    count = grid['width'] * grid['height']
    depth = struct.unpack_from('<' + 'f' * count, payload, products['depth']['offset'])
    validity = payload[products['validity']['offset']:products['validity']['offset'] + count]
    projection = document['calibration_geometry']['matching']['left_projection']
    prefix = subprocess.check_output(['ros2', 'pkg', 'prefix', 'nuvo_camera'], text=True).strip()
    binary = prefix + '/lib/nuvo_camera/nuvo_camera_node'
    rclpy.init()
    with tempfile.TemporaryDirectory(prefix='nuvo-native-wire-') as directory:
        root = pathlib.Path(directory)
        recording = root / 'native.nvr'
        # Wrap the native packet unchanged; the SDK owns NVR1 and wire decoding.
        write_recording(recording, [wire])
        observer = Observer('/native_contract')
        with tempfile.TemporaryFile(mode='w+') as log:
            process = start(binary, '/native_contract', log, source='replay',
                            recording=recording, replay_loop=True,
                            allow_provisional_calibration=True)
            try:
                observer.spin(lambda: observer.ready())
                stamp = max(observer.associated())
                d, v, info, points, metadata = (observer.messages[key][stamp]
                                               for key in observer.messages)
                assert (d.width, d.height) == (grid['width'], grid['height'])
                assert list(v.data) == list(validity)
                assert list(info.p) == projection
                assert metadata.source_capture_time_ns == document['capture_time_ns']
                assert metadata.calibration_sha256 == document['calibration_sha256']
                assert metadata.calibration_authority == document['calibration_authority']
                for key in ('device_uuid', 'run_uuid', 'capture_epoch',
                            'capture_sequence', 'configuration_epoch'):
                    assert getattr(metadata, key) == document[key], key
                for index, value in enumerate(depth):
                    actual_depth = struct.unpack_from('<f', bytes(d.data), index * 4)[0]
                    xyz = struct.unpack_from('<fff', bytes(points.data), index * 12)
                    if validity[index] != 1 or not math.isfinite(value) or value <= 0:
                        assert math.isnan(actual_depth) and all(math.isnan(axis) for axis in xyz)
                        continue
                    u, row = index % d.width, index // d.width
                    expected = ((u - projection[2]) * value / projection[0],
                                (row - projection[6]) * value / projection[5], value)
                    assert actual_depth == value
                    assert all(math.isclose(a, b, rel_tol=1e-6, abs_tol=1e-7)
                               for a, b in zip(xyz, expected)), (xyz, expected)
            except Exception:
                log.flush()
                log.seek(0)
                print(log.read(), file=sys.stderr)
                raise
            finally:
                stop(process)
                observer.node.destroy_node()
    rclpy.shutdown()
    print('Native serialized calibration, identity, depth and XYZ passed over ROS DDS')


if __name__ == '__main__':
    main()
