"""Exercise generic robot integration through installed launch/component APIs.

Two independent synthetic cameras, namespaced topics, explicit optical frames,
topic remapping and ordinary sensor_msgs consumers. No robot package is needed.
"""
from pathlib import Path
import json
import math
import struct
import subprocess
import tempfile
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from integration import Peer, packet, stop


def main():
    rclpy.init()
    observer = rclpy.create_node('generic_robot_observer')
    peers, processes, logs = [], [], []
    received = {}
    try:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for index in range(2):
                namespace = f'/robot/sensors/camera_{index}'
                path = root / f'camera_{index}.sock'
                peers.append(Peer(path, str(index + 2), lambda seq, run: packet(
                    seq, run, live=True, optional_products=True)))
                parameters = dict(source='local', endpoint=str(path),
                                  frame_id=f'camera_{index}_left_optical',
                                  right_frame_id=f'camera_{index}_right_optical',
                                  publish_points=True, point_stride=1,
                                  publish_right_rectified=True)
                received[index] = {key: {} for key in ('depth', 'info', 'points', 'right', 'right_info')}
                remappings = [('depth/image_rect', 'range/image'),
                              ('depth/camera_info', 'range/camera_info'), ('points', 'cloud')]
                topics = [('depth', 'depth/image_rect', Image),
                          ('info', 'depth/camera_info', CameraInfo),
                          ('points', 'points', PointCloud2),
                          ('right', 'right/image_rect', Image),
                          ('right_info', 'right/camera_info', CameraInfo)]
                for key, topic, kind in topics:
                    if index:
                        topic = dict(remappings).get(topic, topic)
                    def record(message, i=index, k=key):
                        stamp = message.header.stamp
                        received[i][k][(stamp.sec, stamp.nanosec)] = message
                    observer.create_subscription(kind, namespace + '/' + topic,
                                                 record, qos_profile_sensor_data)
                if index == 0:
                    # Exact node-scoped parameters exercise the launch name argument.
                    config = root / 'camera.yaml'
                    config.write_text(json.dumps({namespace + '/depth_sensor': {
                        'ros__parameters': parameters}}))
                    command = ['ros2', 'launch', 'nuvo_bringup', 'camera.launch.py',
                               'namespace:=' + namespace, 'name:=depth_sensor',
                               'config:=' + str(config)]
                else:
                    launch = root / 'component.launch.py'
                    launch.write_text('''\
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([ComposableNodeContainer(
        name='camera_container', namespace='/robot',
        package='rclcpp_components', executable='component_container',
        composable_node_descriptions=[ComposableNode(
            package='nuvo_camera', plugin='nuvo_camera::NuvoCamera',
            name='depth_sensor', namespace=%r, parameters=[%r], remappings=%r,
            extra_arguments=[{'use_intra_process_comms': True}])], output='screen')])
''' % (namespace, parameters, remappings))
                    command = ['ros2', 'launch', str(launch)]
                log = open(root / f'camera_{index}.log', 'w+')
                logs.append(log)
                processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                                  start_new_session=True))

            def associated(index):
                return set.intersection(*(set(items) for items in received[index].values()))

            deadline = time.monotonic() + 25
            while time.monotonic() < deadline:
                rclpy.spin_once(observer, timeout_sec=.1)
                assert all(p.poll() is None for p in processes), 'driver launch exited early'
                if all(len(associated(i)) >= 3 for i in received):
                    break
            else:
                raise AssertionError('missing associated sensor_msgs: ' + str({
                    i: {k: len(v) for k, v in items.items()} for i, items in received.items()}))

            for index, messages in received.items():
                stamp = max(associated(index))
                depth, info, points, right, right_info = (v[stamp] for v in messages.values())
                assert all(m.header.frame_id == f'camera_{index}_left_optical'
                           for m in (depth, info, points))
                assert right.header.frame_id == right_info.header.frame_id == f'camera_{index}_right_optical'
                assert depth.encoding == '32FC1' and info.width == depth.width == 2
                z = struct.unpack('<ff', bytes(depth.data))
                assert z[0] == 2. and math.isnan(z[1])
                assert info.p[0] == 100. and right_info.p[3] == -10.
                assert list(right.data) == [27, 28]
                assert points.width == 2 and [field.name for field in points.fields] == ['x', 'y', 'z']
                assert peers[index].subscriptions[-1]['products'] == 42
                assert all(s['preview_cadence_millihz'] == 0 for s in peers[index].subscriptions)
            # The observer uses no Nuvo metadata subscription to obtain associated output.
            print('Two-camera standalone/component launch, remapping, optical frames and standard ROS consumers passed')
    except Exception:
        for log in logs:
            log.flush()
            log.seek(0)
            print(log.read())
        raise
    finally:
        try:
            for process in processes:
                stop(process)
        finally:
            for peer in peers:
                peer.close()
            for log in logs:
                log.close()
            observer.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
