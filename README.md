# Nuvo Depth ROS 2

ROS 2 Jazzy component and executable for native Nuvo Depth on a Raspberry Pi 5.
The default deployment runs Nuvo Depth on Raspberry Pi OS Lite 64-bit and this
driver in an Ubuntu 24.04 Docker container on the same Pi. The container reads
the native data socket; it does not open cameras or run stereo matching.

This is development source. Host tests do not establish Pi 5 hardware,
long-duration performance or metric-accuracy qualification. No ROS APT package
or camera update is implied by this checkout.

## Build and run on the Pi

Install Docker using its maintained Debian instructions. Ensure the native
Nuvo daemon is healthy and capture has an accepted calibration, or explicitly
enabled provisional preview. Use a runtime that provides active matching-grid
geometry. Public camera beta 8 needs the explicit calibration override described
below; installing this driver does not update the camera.

```bash
export NUVO_VIDEO_GID=$(getent group nuvo-video | cut -d: -f3)
docker compose -f docker/compose.yaml build
docker compose -f docker/compose.yaml up
```

The image builds natively on arm64 or amd64. Its user is unprivileged, with the
host's numeric nuvo-video GID added. Mount the entire `/run/nuvo-depth` directory
read-only so a replaced socket remains accessible after a daemon restart. No
camera device, private key, calibration-state directory or privileged container
is required. Use the default Docker time namespace; explicit boottime offsets
are incompatible with local acquisition-time mapping.

Linux host networking exposes DDS discovery on the robot network. Match
`ROS_DOMAIN_ID` across participants. Host IPC allows DDS shared-memory transport
between local containers; Nuvo's memfd socket transport works independently of
DDS shared memory. Network/firewall and selected DDS configuration still govern
discovery. `RMW_IMPLEMENTATION` defaults to Cyclone DDS, which passes the current
container integration checks. Use the same implementation across the initial
robot deployment. Fast DDS is available in the base image but requires separate
discovery/transport qualification; it failed discovery in this WSL test host.

Edit `nuvo_bringup/config/camera.yaml` for namespace-independent parameters.
Point-cloud output is opt-in (`publish_points: true`), computes only while a
subscriber exists, and defaults to every second pixel in each dimension. It
uses the matching calibration and the same rectified optical frame as depth.
Supply the camera-to-robot mounting transform in your robot's URDF/TF tree.
With `lazy: true` (default), acquisition demand is released while no ROS
measurement subscribers exist; diagnostics alone do not run matching.
Set `lazy: false` deliberately when continuous acquisition is required.

## Topics and meaning

Relative to `/nuvo/front` by default:

| Topic | Type and meaning |
|---|---|
| `depth/image_rect` | Image, 32FC1, optical-axis Z metres; rejected/nonfinite/nonpositive samples are NaN |
| `depth/validity` | Image, 8UC1; original Nuvo rejection codes, only 1 means accepted |
| `depth/camera_info` | CameraInfo for the actual matching-grid virtual rectified camera |
| `points` | Optional organized XYZ PointCloud2, metres, invalid NaN, configurable decimation |
| `frame_metadata` | Original identity, calibration authority/hash, source time and timestamp policy |
| `/diagnostics` | Connection/recovery, errors, stale frames, duplicates and calibration/time status |

Depth, validity and CameraInfo share a stamp/frame/grid. CameraInfo describes
an already rectified pinhole model: K from the active left P, R identity and
zero distortion. It makes no claim about raw distorted camera intrinsics.
There is no hardcoded baseline, focal length, resolution or mounting pose.
Provisional authority is retained and requires `allow_provisional_calibration:
true` to publish measurements. Existing validated unit bundles take precedence
in the native runtime; installing/upgrading ROS never triggers calibration.

All measurement topics use best-effort volatile QoS with depth 1. Set
`reliability` and `qos_depth` at startup for a specific DDS/recording deployment.
CameraInfo follows the same policy and is published per frame. Example rosbag2
overrides are in `nuvo_bringup/config/recording-qos.yaml`. Reliable DDS cannot
recover frames already skipped by the native best-effort source.

## Acquisition and replay

`source: local` is the default. Source capture time is CLOCK_BOOTTIME; the driver
maps it to system ROS time using a local bracketed clock sample and reports its
sampling uncertainty. It rejects stale/future local frames and reports system
clock jumps. That uncertainty measures local sampling, not camera hardware
timestamp accuracy. Live simulated ROS time is rejected.

`source: https` uses `endpoint: https://HOST:7443`, `ca_file`, and a restricted
0600 `credential_file`. These are paths; token values never become ROS
parameters. The canonical C++ SDK verifies certificates/hostnames, exchanges a
session, checks product capabilities and streams depth + validity. There is no
remote clock-correlation endpoint today: this mode explicitly uses receipt
timestamps and exposes that limitation in metadata/diagnostics. It is suitable
for acquisition, not a claim of synchronized remote robot perception.
DNS and hosts-file resolution use an interruptible c-ares loop. For mDNS names
resolved by the host, set `connect_address` to that numeric IP while keeping the
device's certificate hostname in `endpoint`.

`source: replay` reads NVR1 recordings, preserving source identity and time.
Set `recording`, `replay_rate` and optional `replay_loop`. Default headers rebase
the source timeline to ROS system time; playback pacing uses a steady clock.
`publish_clock: true` publishes a source-relative simulation timeline on /clock.
Set `use_sim_time: true` on consumers. Only one /clock publisher should be active.
Restarts/loops start a new segment continuing after the preceding ROS stamp;
original source timestamps remain in FrameMetadata. End-of-file stops image
publication while the node remains available for diagnostics. Cancellation
interrupts network reads and playback waits. Transport errors recreate the
source with bounded retry backoff; malformed frames are never republished.

```bash
ros2 run nuvo_camera nuvo_camera_node --ros-args \
  -p source:=replay -p recording:=/recordings/capture.nvr \
  -p publish_points:=true
```

Older recordings or beta 8 lack transport geometry. Use a deliberately prepared
`calibration_override_file` containing the exact `calibration_sha256`, matching
grid and calibrated projections/Q/baseline. The SDK checks the identity and
dimensions, and never replaces producer geometry. See the vendored SDK README
for the JSON contract. Missing geometry produces an explicit diagnostic instead
of fabricated CameraInfo. No override can promote provisional authority.

## Development and repository boundary

`nuvo_depth_client_vendor/upstream` is generated from Nuvo Depth's canonical
`sdk/cpp` and wire-code dependency closure. SOURCE-MANIFEST.json records the
base commit, actual content hashes and license notices. Fix acquisition code
in Nuvo Depth, then run `python3 tools/refresh_sdk.py --source-repo PATH`.
`python3 tools/refresh_sdk.py --check` checks the pinned snapshot without
requiring a sibling repository. Builds never download a moving SDK branch.

In an Ubuntu Jazzy development environment with declared dependencies:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --return-code-on-test-failure
colcon test-result --verbose
source install/setup.bash
ros2 launch nuvo_bringup camera.launch.py
```

The same C++ component can be loaded as `nuvo_camera::NuvoCamera` into a
component container. Parameters are startup-only to avoid acknowledging changes
that the acquisition worker has not applied. Connecting only registers product
demand; it performs no exposure, stereo, calibration or encoder writes.

Routine validation produces CI artifacts. ARM64 package builds and release
promotion remain separate manual/approved operations. The Docker base image
digest is pinned; resolved package versions must also be recorded for a qualified
candidate. Nuvo-authored code and the SDK retain GPL-3.0-or-later licensing.
