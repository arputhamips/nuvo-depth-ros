# Use Nuvo Depth in your ROS project

This is a general-purpose **ROS 2 Jazzy camera driver**. It has no dependency on
Nuvo Base, a particular robot model, navigation stack, controller or robot TF
tree. Applications consume ordinary ROS messages; the optional Nuvo metadata
message adds capture provenance. ROS 1 and other ROS 2 distributions are not
currently tested. The repository's Nuvo-authored code is GPL-3.0-or-later.

## Choose how to connect

| Your setup | Start here |
|---|---|
| Native Nuvo Depth on Pi OS; ROS on the same Pi | [Docker quick start](../README.md#quick-start), the default deployment |
| Existing Ubuntu 24.04 / ROS 2 Jazzy workspace | [Source build](reference.md#source-build-and-development); use the local socket only on the camera's host |
| ROS computer connected to a remote camera | [HTTPS source](reference.md#remote-https); remote headers use receipt time |
| Developing without a camera | [NVR1 replay](reference.md#nvr1-replay), using an existing compatible recording |

Native Nuvo Depth must already have usable calibration and publish the geometry
needed by your selected products. Public camera beta 8 needs the explicit
[matching-grid override](reference.md#beta-8-and-older-recordings) for depth.
Installing this adapter does not install or upgrade the native camera software.
It does not require recalibrating an unchanged camera. Provisional calibration
requires an explicit preview opt-in and retains provisional authority.

## Configure one camera

After building and sourcing the driver, save a parameter file in **your robot's
bringup package** or deployment configuration. For a same-host connection:

```yaml
/**:
  ros__parameters:
    source: local
    endpoint: /run/nuvo-depth/data.sock
    timestamp_mode: local_boottime
    frame_id: depth_camera_left_rectified_optical
    right_frame_id: depth_camera_right_rectified_optical
    cadence_hz: 10.0
    publish_points: true
    point_stride: 2
    lazy: true
    reliability: best_effort
    qos_depth: 1
    allow_provisional_calibration: false
```

`publish_points: true` enables the cloud publisher; XYZ computation still runs
only while it has subscribers. Set it to `false` if you only need depth images.
Parameters are fixed at startup, so restart the node after changing the file.
Add a calibration override only when the source needs it.

```bash
ros2 launch nuvo_bringup camera.launch.py \
  namespace:=sensors/depth_camera name:=depth_sensor \
  config:=/absolute/path/to/your/camera.yaml
```

For Compose, put your YAML in `nuvo_bringup/config` and add these settings to
`docker/.env` alongside `NUVO_VIDEO_GID` and `ROS_DOMAIN_ID`:

```dotenv
NUVO_NAMESPACE=sensors/depth_camera
NUVO_CONFIG=/config/my-camera.yaml
```

Then recreate the service using the [README command](../README.md#troubleshooting).
The configuration path is inside the container. Topic namespaces and frame IDs
are independent: choosing a namespace does not rename frames.

## Receive standard ROS topics

With the namespace above, these are the usual inputs to a robot application:

| Input | Topic | Contract |
|---|---|---|
| Depth | `/sensors/depth_camera/depth/image_rect` | `sensor_msgs/Image`, `32FC1`, optical-axis Z in metres, invalid NaN |
| Depth intrinsics | `/sensors/depth_camera/depth/camera_info` | `sensor_msgs/CameraInfo`, calibration for the actual depth grid |
| Optional XYZ | `/sensors/depth_camera/points` | `sensor_msgs/PointCloud2`, `x/y/z` in metres, invalid NaN |
| Validity detail | `/sensors/depth_camera/depth/validity` | `sensor_msgs/Image`, `8UC1`, only code `1` accepted |

Images, CameraInfo and clouds from a capture have the same timestamp. Pair an
image with its CameraInfo by stamp; don't use assumed dimensions, a raw camera
matrix or the stereo image's CameraInfo for a different depth grid. The driver
also supports optional rectified stereo and disparity; see [all topics](../README.md#topics).

Use best-effort subscriber QoS (C++ `rclcpp::SensorDataQoS()` or Python
`rclpy.qos.qos_profile_sensor_data`) with the default publishers. A subscriber
requiring reliable delivery cannot connect to a best-effort publisher.

```bash
ros2 topic echo /sensors/depth_camera/depth/image_rect \
  --once --field header --qos-reliability best_effort
ros2 topic echo /sensors/depth_camera/depth/camera_info \
  --once --qos-reliability best_effort
```

These two CLI checks confirm individual topics; they do not synchronize the two
captures. Listing topics alone does not request data because acquisition is lazy.
An image, CameraInfo or point-cloud subscription activates the required products.

For a minimal Python consumer in an already sourced environment:

```python
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

rclpy.init()
node = rclpy.create_node('depth_consumer')
subscription = node.create_subscription(
    Image, '/sensors/depth_camera/depth/image_rect',
    lambda msg: node.get_logger().info(
        f'{msg.width}x{msg.height} {msg.encoding} frame={msg.header.frame_id}'),
    qos_profile_sensor_data)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
```

Save this in your own project and run it with `python3`. No Nuvo-specific message
type is needed. A geometry-processing consumer should additionally subscribe to
CameraInfo and synchronize by timestamp. Consumers requiring millimetre `16UC1`,
RGB-aligned depth or compressed image transports need an explicit downstream
conversion; those outputs are not provided by this driver.

In RViz2, add a PointCloud2 display for `points`, set its reliability to
**Best Effort**, and initially use `depth_camera_left_rectified_optical` as the
fixed frame. This requires `publish_points: true`. To view the cloud in a robot
frame, first provide the mounting transforms described below. RViz2 is an
optional desktop tool; it is not installed in the headless driver container.

## Add it to your robot launch file

Keep your camera mount, topic remaps and application configuration in your own
package. A standard `launch_ros.actions.Node` supports arbitrary ROS remapping:

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nuvo_camera', executable='nuvo_camera_node',
            namespace='sensors/depth_camera', name='depth_sensor',
            parameters=[{
                'source': 'local',
                'endpoint': '/run/nuvo-depth/data.sock',
                'frame_id': 'depth_camera_left_rectified_optical',
                'right_frame_id': 'depth_camera_right_rectified_optical',
                'publish_points': True,
            }],
            remappings=[
                ('depth/image_rect', 'range/image'),
                ('depth/camera_info', 'range/camera_info'),
                ('points', 'cloud'),
            ],
            output='screen'),
    ])
```

Add `nuvo_camera` as an `exec_depend` in that package's `package.xml`. The image
in this example becomes `/sensors/depth_camera/range/image`. Remap image and
CameraInfo together. `/diagnostics` is absolute and shared by default; remap it
explicitly if isolation is needed. Each diagnostic status names its ROS node.

For a component container, use `launch_ros.descriptions.ComposableNode` with
`package='nuvo_camera'`, `plugin='nuvo_camera::NuvoCamera'`, and the same name,
namespace, parameters and remappings. Both the executable and component use
the same driver. [C++ linking](reference.md#source-build-and-development) is
also available for applications that instantiate the node directly.

For multiple cameras, create one node per source, each with its own endpoint,
namespace and left/right optical frame IDs. Use distinct device pins if required.
Each local endpoint must be supplied by a native daemon; multiple ROS nodes
pointing to one socket subscribe to the same camera. Remote cameras use separate
HTTPS configurations. Multiple sources do not imply synchronized capture.

## Supply your robot's transforms and behavior

The published frame is a **rectified optical camera frame**: X right, Y down,
Z forward. The adapter does not publish TF. Your URDF/TF tree supplies the
calibrated mounting relationship to your robot and, when used, the right optical
frame. A namespace or frame name is not a transform, and a rectified frame must
not be assumed identical to a raw sensor frame without its calibration.

| Owner | Responsibilities |
|---|---|
| Native Nuvo Depth | Capture, exposure, stereo matching, camera settings, calibration and its authority |
| This ROS driver | Acquisition subscriptions, standard ROS messages, timestamp policy, optional XYZ and measurement diagnostics |
| Your robot package | Mounting TF/URDF, navigation, obstacle interpretation, control, safety decisions and consumer deadlines |

Use [the integration guide](integration.md) for device/calibration pins, failure
policies and commissioning. Hardware accuracy and sustained Pi performance
still need testing on the actual camera and robot; host fixtures do not establish
those properties.
