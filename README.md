# Nuvo Depth ROS 2

ROS 2 Jazzy driver that publishes [Nuvo Depth](https://github.com/arputhamips/nuvo-depth)
output as depth images, validity and CameraInfo, with optional XYZ, rectified
stereo images and disparity.
The default setup runs ROS in Docker on the same Raspberry Pi 5 as the native
Nuvo daemon; it leaves camera capture, matching and calibration to Nuvo Depth.

This is a source package, with no released ROS APT package or prebuilt image.
[Validation records](docs/validation-20261026.md) describe the host/container evidence; ARM64 builds, Pi camera
operation, sustained performance and metric accuracy still need qualification.
Use the [integration guide](docs/integration.md) when replacing an existing
camera adapter or commissioning a downstream consumer.

## Quick start

Run these commands on the **Pi host**, locally or over SSH. You need:

- Raspberry Pi OS Lite 64-bit with native Nuvo Depth already working, using the
  [full](https://docs.nuvohardware.com/vision-one/install/byo-pi) or
  [minimal](https://docs.nuvohardware.com/vision-one/install/minimal) installation.
- Git, [Docker Engine and the Compose plugin](https://docs.docker.com/engine/install/debian/).
  ROS runs inside the Ubuntu 24.04 / Jazzy container; no host ROS install is needed.
- The unit's existing calibration. Preserve validated per-unit bundles; a ROS
  installation or software upgrade does not require recalibration.
- **Public camera beta 8: prepare the [matching geometry override](docs/reference.md#beta-8-and-older-recordings)
  before starting.** Beta 8 does not send the geometry this driver requires.
  Missing geometry blocks measurement publication. Provisional calibration also
  needs the explicit `allow_provisional_calibration: true` opt-in.

Check the prerequisites:

```bash
uname -m                                      # expected: aarch64
systemctl is-active nuvo-depthd-native.service # expected: active
sudo test -S /run/nuvo-depth/data.sock && echo "Nuvo socket exists"
getent group nuvo-video                       # must return a numeric GID
sudo docker version                          # client and server must respond
sudo docker compose version
```

Clone the driver and save the host's numeric group ID:

```bash
git clone https://github.com/arputhamips/nuvo-depth-ros.git
cd nuvo-depth-ros
NUVO_VIDEO_GID="$(getent group nuvo-video | cut -d: -f3)"
test -n "$NUVO_VIDEO_GID" || { echo "Missing nuvo-video group" >&2; exit 1; }
printf 'NUVO_VIDEO_GID=%s\nROS_DOMAIN_ID=0\n' "$NUVO_VIDEO_GID" > docker/.env
```

The ignored `docker/.env` persists these settings across terminals. Change
`ROS_DOMAIN_ID` there to match your robot. Run all Compose commands from this
repository root; each explicitly loads that file.

Edit [`nuvo_bringup/config/camera.yaml`](nuvo_bringup/config/camera.yaml) if you
need the geometry override, provisional preview or point clouds. Its directory
is mounted at `/config`: a prepared host file `nuvo_bringup/config/geometry.json`
uses `calibration_override_file: /config/geometry.json` in the YAML.

Build the image and start the driver:

```bash
sudo docker compose --env-file docker/.env -f docker/compose.yaml config --quiet
sudo docker compose --env-file docker/.env -f docker/compose.yaml build
sudo docker compose --env-file docker/.env -f docker/compose.yaml up -d
sudo docker compose --env-file docker/.env -f docker/compose.yaml logs --tail=50 camera
```

The build runs colcon tests and defaults to two compiler jobs. The container
runs unprivileged, using the saved `nuvo-video` group and a read-only mount of
`/run/nuvo-depth`; camera-device mounts and privileged mode are unnecessary.
The supplied [Cyclone DDS profile](docs/reference.md#dds-transport) keeps UDP
datagrams within a standard 1500-byte MTU for image and cloud delivery.

Receive a depth frame using the container's ROS environment:

```bash
sudo docker compose --env-file docker/.env -f docker/compose.yaml exec camera \
  bash -lc 'source /opt/ros/jazzy/setup.bash &&
    source /ws/install/setup.bash &&
    ros2 topic echo /nuvo/front/depth/image_rect --once --field header --qos-reliability best_effort'
```

A received header contains a timestamp and frame ID
`nuvo_front_left_rectified_optical`. **The driver is lazy by default:** a
measurement subscriber starts acquisition. Listing topics or reading diagnostics
does not request frames; demand is released after the last subscriber leaves.
The command uses best-effort QoS to match the publisher.

## Configuration

Parameters in [`camera.yaml`](nuvo_bringup/config/camera.yaml) are fixed at
startup. After editing the file, reload them with:

```bash
sudo docker compose --env-file docker/.env -f docker/compose.yaml restart camera
```

| Parameter | Default in `camera.yaml` | Purpose |
| --- | --- | --- |
| `cadence_hz` | `10.0` | Requested delivery cadence, not a guaranteed frame rate. |
| `publish_points` | `false` | Enable XYZ; conversion runs only with cloud subscribers. |
| `publish_left_rectified` / `publish_right_rectified` | `false` / `false` | Enable rectified image and CameraInfo publishers; native products are requested only with subscribers. |
| `publish_disparity` | `false` | Enable calibrated `stereo_msgs/DisparityImage` output. |
| `point_stride` | `2` | Sample every second pixel in each dimension for XYZ; `1` uses the full grid. |
| `lazy` | `true` | Release acquisition demand without measurement subscribers. |
| `allow_provisional_calibration` | `false` | Explicitly allow provisional preview while preserving its authority. |
| `reliability` / `qos_depth` | `best_effort` / `1` | Measurement publisher QoS; match subscribers and recording settings. |

Optional device/calibration pins and restart/clock policies are described in the
[reference](docs/reference.md#identity-calibration-and-clock-policies).
The default leaves pins empty, accepts acquisition restarts and remaps clock
jumps. Use the [strict example](nuvo_bringup/config/strict.yaml) for commissioning.

The default namespace is `/nuvo/front`; `frame_id` is
`nuvo_front_left_rectified_optical`. Supply its mounting transform in your robot's
URDF/TF tree. The launch arguments are `namespace` and `config`; see the
[reference](docs/reference.md) for source builds and other launch configurations.

## Topics

Measurements use best-effort, volatile QoS with depth 1 by default.

| Topic under `/nuvo/front` | Message type | Meaning |
| --- | --- | --- |
| `depth/image_rect` | `sensor_msgs/msg/Image` | `32FC1`, optical-axis Z in metres; invalid samples are NaN. |
| `depth/validity` | `sensor_msgs/msg/Image` | `8UC1`, original rejection codes; only `1` means accepted. |
| `depth/camera_info` | `sensor_msgs/msg/CameraInfo` | Actual matching-grid rectified camera geometry, per frame. |
| `points` | `sensor_msgs/msg/PointCloud2` | Optional organized XYZ in metres, invalid points NaN. |
| `left/image_rect`, `left/camera_info` | `sensor_msgs/msg/Image`, `sensor_msgs/msg/CameraInfo` | Optional rectified left image and its image-grid calibration. |
| `right/image_rect`, `right/camera_info` | Same as left | Optional right image and calibration, using `right_frame_id`. |
| `disparity` | `stereo_msgs/msg/DisparityImage` | Optional matching-grid disparity with calibrated focal length, baseline and producer limits. |
| `frame_metadata` | `nuvo_camera_msgs/msg/FrameMetadata` | Capture identity, calibration hash/authority, source time and timestamp policy. |
| `/diagnostics` (absolute) | `diagnostic_msgs/msg/DiagnosticArray` | Connection, freshness, calibration and time status; reliable QoS. |

Depth, validity and CameraInfo share their timestamp, frame and grid. See
[measurement semantics](docs/reference.md#image-cloud-and-metadata-semantics)
for projection, validity and timestamp details.
Diagnostics describe fresh measurement availability and calibration authority;
they do not establish robot health. Keep provisional evaluation in the separate
[inspection launch](docs/integration.md#isolated-inspection).

## Optional: local-only operation

Local ROS acquisition uses read-only `memfd` frames from the Unix socket and
never falls back to HTTPS. ROS message construction and DDS may copy data;
this is not end-to-end zero-copy. Compose's host networking serves **ROS DDS**,
which still depends on the robot's network and firewall configuration.

The Nuvo gateway is unnecessary for this path. To stop browser and remote SDK
access on an existing full installation, run on the **Pi host**:

```bash
sudo systemctl mask --now nuvo-gateway.service
systemctl is-active nuvo-depthd-native.service # expected: active
```

Masking prevents the appliance target from starting the gateway; disabling
alone is insufficient. **The full installer/setup enables and restarts the
gateway. Unmask it before rerunning that workflow, then reapply the mask after
successful setup if the device should stay local-only.** The minimal profile
already omits it. Keep networking for DDS and SSH. See
[service details and restoration](docs/reference.md#local-only-operation).

## Troubleshooting

| Symptom | Check |
| --- | --- |
| Node runs but no images arrive | Start a measurement subscriber with best-effort QoS; topic discovery alone does not activate the lazy driver. |
| Missing geometry or provisional-calibration diagnostic | Follow the [beta 8 compatibility instructions](docs/reference.md#beta-8-and-older-recordings). An override must match the actual hash/grid and cannot validate provisional calibration. |
| Socket missing or permission denied | Check the native service, socket and `nuvo-video` GID. Older native units or manual runtime-directory removal can leave a stale container mount. |
| Another ROS machine cannot discover topics | Match `ROS_DOMAIN_ID`, QoS and the robot's DDS/network settings. The Nuvo gateway does not carry DDS traffic. |

Updated native full/minimal units preserve the runtime directory across ordinary
restarts. After upgrading older units or replacing that directory, wait for native
capture to be active and recreate the container. Also recreate after changing
`docker/.env`:

```bash
sudo docker compose --env-file docker/.env -f docker/compose.yaml up -d --force-recreate camera
```

Use the logs command above to inspect driver messages. To read why publication
is blocked, reuse the frame-check command and replace only `ros2 topic echo ...`
with `ros2 topic echo /diagnostics --once`.

See the [driver reference](docs/reference.md) for HTTPS, NVR1 replay, rosbag2 QoS,
timing, components, source builds and the generated SDK boundary.
Nuvo-authored code is [GPL-3.0-or-later](LICENSE); see [NOTICE](NOTICE) for scope.
