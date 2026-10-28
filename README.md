# Nuvo Depth ROS 2

ROS 2 Jazzy driver that publishes [Nuvo Depth](https://github.com/arputhamips/nuvo-depth)
output as depth images, validity and CameraInfo, with optional XYZ, rectified
stereo images and disparity.
It has no dependency on Nuvo Base or any robot-specific control/navigation stack.
Applications can use standard ROS messages without the optional Nuvo metadata.
The default setup runs ROS in Docker on the same Raspberry Pi 5 as the native
Nuvo daemon; it leaves camera capture, matching and calibration to Nuvo Depth.

The [v0.1.3 release](https://github.com/arputhamips/nuvo-depth-ros/releases/tag/v0.1.3)
provides source and a [prebuilt Docker image](docs/docker-image.md) for Pi ARM64
and Linux PC AMD64. There is no released ROS APT package; local source builds
remain supported. See the [changelog](CHANGELOG.md) for features and limitations.
[Release validation](docs/validation-20261028.md) covers AMD64/ARM64 builds,
container checks, public downloads, public installation and 25 camera argument
cases on the Pi. Actual Pi
delivery exposed a small host DDS receive-buffer cap; v0.1.3 configures the
needed minimum automatically. Sustained performance and metric accuracy still
need qualification. The earlier [Pi record](docs/validation-pi5-20261027.md)
covers v0.1.1 and its own tested cohort.
The test Pi's four native startup retries were traced to SD/MMC I/O stalls.
See [startup diagnostics](docs/native-startup.md#slow-boot-or-repeated-trigger-startup-timeouts)
before changing camera settings. A device-specific SD workaround passed two
subsequent reboots in about 53 seconds with zero native retries.
The prebuilt image includes startup output/rate controls and explicit native
calibration-bundle selection; no source build is needed. Native
`v0.1.1-beta.9` is the compatible full/minimal camera release for these controls.
New to the driver? Follow [Use Nuvo Depth in your ROS project](docs/using-in-robot.md)
for custom namespaces, frames, topic remaps, a Python subscriber, RViz and multiple
cameras. Use the [integration guide](docs/integration.md) for commissioning.

See [how the same-Pi, Ethernet and USB-C paths communicate](docs/connections.md).

See [startup controls for robot prototyping](docs/prototyping.md) for output
selection, requested cadence, namespaces and explicit calibration-bundle selection.

Choose the [Pi Docker setup](#quick-start), an [Ubuntu Jazzy source build](docs/reference.md#source-build-and-development),
a [remote HTTPS camera](docs/reference.md#remote-https), or [recording replay](docs/reference.md#nvr1-replay).
ROS 1 and other ROS 2 distributions are not currently tested.

## Quick start

Run these commands on the **Pi host**, locally or over SSH. You need:

- Raspberry Pi OS Lite 64-bit with native Nuvo Depth already working, using the
  [full](https://docs.nuvohardware.com/vision-one/install/byo-pi) or
  [minimal](https://docs.nuvohardware.com/vision-one/install/minimal) installation.
  The beta 9 [full installer](https://github.com/arputhamips/nuvo-depth/releases/download/v0.1.1-beta.9/install-nuvo-depth.sh)
  and [minimal installer](https://github.com/arputhamips/nuvo-depth/releases/download/v0.1.1-beta.9/install-nuvo-depth-minimal.sh)
  are pinned release downloads; upgrade within the profile already installed.
- Python 3, curl, tar, [Docker Engine and the Compose plugin](https://docs.docker.com/engine/install/debian/).
  ROS runs inside the Ubuntu 24.04 / Jazzy container; no host ROS install is needed.
- The unit's existing calibration. Preserve validated per-unit bundles; a ROS
  installation or software upgrade does not require recalibration.
- Native **v0.1.1-beta.9** in the profile already installed. It supplies geometry,
  runtime permissions and local calibration control. Its supported Pi kernel is
  **`6.12.75+rpt-rpi-2712`**; check with `uname -r` before installation.
  Older beta 8 needs the
  [matching geometry override](docs/reference.md#beta-8-and-older-recordings)
  for depth and does not support explicit bundle activation through ROS.

If native startup fails, resolve it using [native startup and GPIO/PWM recovery](docs/native-startup.md)
before starting ROS. A socket left by an old process is insufficient evidence.

```bash
uname -m                                      # expected: aarch64
systemctl is-active nuvo-depthd-native.service # expected: active
sudo test -S /run/nuvo-depth/data.sock && echo "Nuvo socket exists"
sudo docker version                          # client and server must respond
sudo docker compose version
```

Download and run the fixed release installer:

```bash
curl -fsSL https://raw.githubusercontent.com/arputhamips/nuvo-depth-ros/v0.1.3/install-nuvo-depth-ros.sh \
  | sudo bash -s -- --outputs depth,points --cadence-hz 10
```

This pulls `ghcr.io/arputhamips/nuvo-depth-ros:jazzy-0.1.3`, downloads deployment
files at the image's recorded source revision into `/opt/nuvo-depth-ros/releases/`
and pins the container to that image digest. The existing helper discovers the
numeric `nuvo-video` group, checks native health and starts Compose. No build,
Git checkout or manual `.env` setup is required. Docker itself is a prerequisite;
the script reports a missing Engine/Compose installation without installing it.

The helper raises the host's `net.core.rmem_max` receive-buffer limit to at
least **8 MiB**, preserving higher values. Its root-owned
`nuvo-depth-ros-network.service` reapplies that conditional minimum after normal
sysctl configuration and before Docker at boot. Default socket buffers stay
unchanged. Each actual installer/helper run recreates the ROS container so DDS
opens fresh sockets, including retries after a failed start. Native capture,
camera configuration and calibration stay under native Nuvo Depth's control.

The first run saves safe defaults in `/etc/nuvo-depth-ros/camera.yaml`; reruns
preserve that file. Omitted output/rate options use the YAML. Add
`--namespace robot/front --ros-domain-id 42` for your robot, or
`--config /absolute/path/to/camera.yaml` to use an existing file. Pass the same
options on later reconfiguration runs; the deployed container retains them
across automatic restarts. `--dry-run` checks readiness and prints the resolved
deployment without starting ROS, activating calibration or changing host buffer
configuration. The bootstrap can still pull/cache its image and source and
create the initial parameter file; the helper's host setup is read-only in this mode.

For provisional inspection, add **`--allow-provisional-calibration`** explicitly.
The default blocks provisional measurement publication and leaves
`calibration_file` empty, preserving the native selection. A validated per-unit
calibration does not need that preview flag. See the
[bundle-selection example](docs/prototyping.md#select-an-existing-calibration-bundle)
only when you intend to change native calibration.

To review the script before running it, save the same versioned URL:

```bash
curl -fsSL -o install-nuvo-depth-ros.sh \
  https://raw.githubusercontent.com/arputhamips/nuvo-depth-ros/v0.1.3/install-nuvo-depth-ros.sh
less install-nuvo-depth-ros.sh
sudo bash install-nuvo-depth-ros.sh --outputs depth,points --cadence-hz 10
sudo docker logs --tail=50 docker-camera-1
```

The image automatically selects ARM64 on the Pi or AMD64 on a Linux PC. It
runs unprivileged, using the discovered `nuvo-video` group and a read-only mount of
`/run/nuvo-depth`; camera-device mounts and privileged mode are unnecessary.
The supplied [Cyclone DDS profile](docs/reference.md#dds-transport) keeps UDP
datagrams within a standard 1500-byte MTU and requests an 8 MiB receive buffer
for image/cloud bursts. Cyclone DDS consumers also need that profile; subscriber
computers need their own suitable host receive limit. The camera container
loads the profile automatically.

Receive a depth frame using the container's ROS environment:

```bash
sudo docker exec docker-camera-1 \
  /nuvo-entrypoint.sh ros2 topic echo /nuvo/front/depth/image_rect --once --field header --qos-reliability best_effort
```

A received header contains a timestamp and frame ID
`nuvo_front_left_rectified_optical`. **The driver is lazy by default:** a
measurement subscriber starts acquisition. Listing topics or reading diagnostics
does not request frames; demand is released after the last subscriber leaves.
The command uses best-effort QoS to match the publisher.

Inspect the driver status without requesting camera processing:

```bash
sudo docker exec docker-camera-1 \
  /nuvo-entrypoint.sh ros2 topic echo /diagnostics --once
```

After a valid stream loses its last measurement subscriber, expect
`waiting for ROS measurement subscribers` and `output_available: false`.
Existing identity, clock or calibration failures remain visible. With
provisional calibration and no opt-in, a
measurement subscriber instead receives a diagnostic explaining the calibration
gate. Keep that authority visible; installing ROS does not validate distances.

See the [Docker guide](docs/docker-image.md) for updating an existing deployment,
remote-camera containers and [optional local builds](docs/docker-image.md#optional-build-from-source).

## Configuration

Parameters in [`camera.yaml`](nuvo_bringup/config/camera.yaml) are fixed at
startup. For the bootstrap installation, edit `/etc/nuvo-depth-ros/camera.yaml`
and rerun the saved installer with the same launch options:

```bash
sudo bash install-nuvo-depth-ros.sh --outputs depth,points --cadence-hz 10
```

| Parameter | Default in `camera.yaml` | Purpose |
| --- | --- | --- |
| `cadence_hz` | `10.0` | Requested delivery cadence, not a guaranteed frame rate. |
| `publish_depth` | `true` | Depth, validity and matching CameraInfo publishers; independent of optional outputs. |
| `publish_points` | `false` | Enable XYZ; conversion runs only with cloud subscribers. |
| `publish_left_rectified` / `publish_right_rectified` | `false` / `false` | Enable rectified image and CameraInfo publishers; native products are requested only with subscribers. |
| `publish_disparity` | `false` | Enable calibrated `stereo_msgs/DisparityImage` output. |
| `point_stride` | `2` | Sample every second pixel in each dimension for XYZ; `1` uses the full grid. |
| `lazy` | `true` | Release acquisition demand without measurement subscribers. |
| `allow_provisional_calibration` | `false` | Explicitly allow provisional preview while preserving its authority. |
| `calibration_file` | `""` | Empty retains native calibration; an explicit complete native bundle delegates validation/activation to native. |
| `reliability` / `qos_depth` | `best_effort` / `1` | Measurement publisher QoS; match subscribers and recording settings. |

Optional device/calibration pins and restart/clock policies are described in the
[reference](docs/reference.md#identity-calibration-and-clock-policies).
The default leaves pins empty, accepts acquisition restarts and remaps clock
jumps. Use the [strict example](nuvo_bringup/config/strict.yaml) for commissioning.

The default namespace is `/nuvo/front`; `frame_id` is
`nuvo_front_left_rectified_optical`. Supply its mounting transform in your robot's
URDF/TF tree. Launch also exposes output/rate and calibration controls; see
[startup controls](docs/prototyping.md) and the [reference](docs/reference.md).

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
| Native fails to request the trigger GPIO line | Follow [GPIO/PWM recovery](docs/native-startup.md#gpio-17-is-already-owned-by-pwm); upgrade the native startup components together. ROS does not own GPIO. |
| Idle v0.1.0 reports `no_usable_measurement_products` | Upgrade to the prebuilt v0.1.3 image. Disconnection should report waiting for subscribers and `output_available=false`. |
| Missing geometry or provisional-calibration diagnostic | Follow the [beta 8 compatibility instructions](docs/reference.md#beta-8-and-older-recordings). An override must match the actual hash/grid and cannot validate provisional calibration. |
| Socket missing or permission denied | Check the native service, socket and `nuvo-video` GID. Older native units or manual runtime-directory removal can leave a stale container mount. |
| Images lose frames despite native delivery | Rerun the v0.1.3 bootstrap/helper with the same options to prepare the host DDS receive limit; inspect `systemctl status nuvo-depth-ros-network.service`. Each remote subscriber host needs suitable DDS buffers too. |
| Another ROS machine cannot discover topics | Match `ROS_DOMAIN_ID`, QoS and the robot's DDS/network settings. The Nuvo gateway does not carry DDS traffic. |

Updated native full/minimal units preserve the runtime directory across ordinary
restarts. After upgrading older units or replacing that directory, wait for native
capture to be active and rerun the installer with your launch options. For a
manual checkout deployment, recreate after changing `docker/.env`:

```bash
sudo docker compose --env-file docker/.env -f docker/compose.yaml up -d --force-recreate camera
```

Use the logs command above to inspect driver messages. To read why publication
is blocked, reuse the frame-check command and replace only `ros2 topic echo ...`
with `ros2 topic echo /diagnostics --once`.

See the [driver reference](docs/reference.md) for HTTPS, NVR1 replay, rosbag2 QoS,
timing, components, source builds and the generated SDK boundary.
Nuvo-authored code is [GPL-3.0-or-later](LICENSE); see [NOTICE](NOTICE) for scope.
