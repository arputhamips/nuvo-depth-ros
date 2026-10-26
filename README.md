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

The image builds natively on arm64 or amd64. Builds default to two compiler jobs
to bound memory use on the Pi; `BUILD_JOBS` is an optional Docker build argument.
Its user is unprivileged, with the
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

## Same-Pi shared memory and network isolation

The default `source: local` reads `/run/nuvo-depth/data.sock`. The native daemon
passes a read-only `memfd` handle over a Unix-domain socket (`SCM_RIGHTS`), and
the C++ client maps that memory directly. The socket carries small control and
frame-descriptor messages, not the depth pixel payload. There is no TCP, HTTP,
WebSocket, TLS, disk-frame polling or gateway in this acquisition path.
Local connection failures retry the local socket; they never fall back to HTTPS.
The bounded wait wakes when a new frame is available; its timeout is not an
added per-frame delay. Docker needs the runtime-directory mount and numeric
`nuvo-video` group, not shared `/dev/shm` or host networking for this handoff.

```text
Native depthd -> sealed frame memory -> mapped C++ client -> ROS messages -> DDS
```

This is not end-to-end zero-copy: the daemon packs selected products into a
cached frame buffer, the adapter fills ROS-owned image arrays and normalizes
invalid depth to NaN, and downstream DDS may serialize/copy messages. The SDK
does not copy the mapped pixel buffer. The ROS subscription requests only depth
and validity, with zero encoded-preview demand. Optional XYZ runs only with
subscribers; best-effort queue depth 1 avoids building a delivery backlog.

The native service itself is restricted to `AF_UNIX`. Nuvo's separate gateway
publishes frames only for explicit network consumers and releases native demand
when they disconnect. Local ROS acquisition does not start network streaming or
encoding. An open browser preview or remote SDK subscriber is additional demand
and can still use CPU and bandwidth while the gateway is enabled.

For a new headless ROS appliance, the existing Nuvo Depth **minimal** installation
omits the gateway, web console and encoder entirely while retaining the same
native data socket. Existing full installations can keep their package and
calibration state and explicitly turn off Nuvo's gateway on the **Pi host**:

```bash
sudo systemctl mask --now nuvo-gateway.service
sudo systemctl is-active nuvo-depthd-native.service
sudo systemctl is-enabled nuvo-gateway.service  # expected: masked
```

Masking prevents the appliance target from starting the gateway again; disabling
alone is insufficient because that target explicitly wants it. Web-console and
remote SDK access are unavailable while it is masked. Restore them when needed:

```bash
sudo systemctl unmask nuvo-gateway.service
sudo systemctl start nuvo-gateway.service
```

The full installer/setup enables and restarts the gateway. Unmask it before
rerunning that workflow, then reapply the mask after successful setup if the
device should stay local-only. This is an operator service override, not a new
installation profile.

Keep the host network available for ROS DDS and SSH. Compose uses
`network_mode: host` for ROS discovery/publication; it does not route Nuvo camera
acquisition through the network or start the host gateway. Do not migrate an
existing full installation to minimal by removing packages; follow Nuvo Depth's
supported installation/profile guidance. Beta 8 still requires the explicit
geometry override described below, regardless of package profile.

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
Blocked live streams release acquisition demand and probe every five seconds
for validated calibration. Blocked replay retains its normal playback pacing.

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
timestamp accuracy. Live simulated ROS time is rejected; `use_sim_time` is
fixed at startup for every source.

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
Remote `cadence_hz` must be at least 1/60 Hz (one frame per minute), so the
message deadline can cover two frame intervals within the SDK's 120-second
limit. Local acquisition retains the SDK's 0.001 Hz minimum.

`source: replay` reads NVR1 recordings, preserving source identity and time.
Set `recording`, `replay_rate` and optional `replay_loop`. Default headers rebase
the source timeline to ROS system time; playback pacing uses a steady clock.
Lazy replay pauses that pacing and retains its file position while measurement
subscribers are absent. It resumes without rewinding or a catch-up burst.
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
