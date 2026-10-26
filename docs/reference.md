# Driver reference

Start with the [README](../README.md) for the same-Pi Docker setup. This page
covers other sources, calibration compatibility and development. Commands below
run in a sourced ROS 2 Jazzy environment; container paths must exist inside the
container.

## Local-only operation

The default `source: local` reads `/run/nuvo-depth/data.sock`. The daemon passes
a read-only sealed `memfd` over a Unix socket; the SDK maps the pixel buffer.
The socket carries control messages and frame descriptors. Local acquisition
uses no HTTP, TLS, frame-file polling or gateway, and a failed local connection
retries the local socket.

Mount the entire runtime directory to follow socket replacement. Updated native
full and minimal service units use `RuntimeDirectoryPreserve=yes`, keeping the
directory inode across restart, stop/start and failed startup. Their cleanup
removes generated socket/status/snapshot artifacts while preserving calibration
and persistent state.

Older installed service units can remove and recreate `/run/nuvo-depth`, leaving
an existing container bound to the old directory. After upgrading from those
units, or after manual runtime-directory replacement, wait for native capture
to be active and recreate the camera container once from the repository root:

```bash
sudo docker compose --env-file docker/.env -f docker/compose.yaml up -d --force-recreate camera
```

The host's numeric `nuvo-video` GID grants access. This handoff needs neither a
camera-device mount nor shared `/dev/shm`. Compose uses host networking for ROS
DDS and host IPC for DDS shared memory; these are separate from Nuvo's `memfd`
handoff. Keep networking available for DDS and SSH.

Stop the adapter, or start it again later:

```bash
sudo docker compose --env-file docker/.env -f docker/compose.yaml stop
sudo docker compose --env-file docker/.env -f docker/compose.yaml up -d
```

The SDK maps the frame without copying its pixel buffer, but the complete path
also fills ROS-owned image arrays and may serialize/copy through DDS. The default
ROS subscription requests depth and validity with no encoded-preview demand.
Enabled optional products are requested only while their image, CameraInfo or
disparity publishers have subscribers. Disparity also requests native validity
so rejected samples can be masked. Changing demand reconnects the live
subscription without resetting identity or clock policy state.

For a new headless appliance, Nuvo Depth's minimal installation omits the
gateway, web console and encoder while keeping the native data socket. On an
existing full installation, run these commands on the **Pi host** to keep its
package and calibration state while stopping network preview and remote SDK
access:

```bash
sudo systemctl mask --now nuvo-gateway.service
sudo systemctl is-active nuvo-depthd-native.service  # expected: active
sudo systemctl is-enabled nuvo-gateway.service      # expected: masked
```

Masking prevents the appliance target from starting the gateway again;
disabling alone is insufficient because that target explicitly wants it.
Restore web-console and remote SDK access when needed:

```bash
sudo systemctl unmask nuvo-gateway.service
sudo systemctl start nuvo-gateway.service
```

The full installer/setup enables and restarts the gateway. Unmask it before
rerunning that workflow, then reapply the mask after successful setup if the
device should stay local-only. Follow Nuvo Depth's supported installation/profile
guidance for package changes. The geometry requirement below applies to beta 8
with either package profile.

## Remote HTTPS

See [connection paths](connections.md) for Ethernet and provisioned USB Ethernet,
and the [remote Docker setup](docker-image.md#remote-ethernet-or-provisioned-usb-ethernet)
for ready-to-use mounts and credential ownership.

Remote acquisition requires the Nuvo gateway on the camera host. Save a separate
configuration file, for example `/path/to/https.yaml`:

```yaml
/**:
  ros__parameters:
    source: https
    endpoint: https://camera.example:7443
    timestamp_mode: receipt
    ca_file: /secrets/nuvo-ca.pem
    credential_file: /secrets/nuvo-token
    frame_id: nuvo_front_left_rectified_optical
    cadence_hz: 10.0
    delivery: best_effort
```

```bash
ros2 launch nuvo_bringup camera.launch.py config:=/path/to/https.yaml
```

Use the certificate's hostname in `endpoint`. The URL accepts a host and optional
port, without a base path, credentials or query string. `ca_file` supplies the
trusted CA; omit it to use system trust. `credential_file` contains the existing
Nuvo token and must be a regular file readable by the driver with owner-only
permissions (`chmod 600`). The token itself is never a ROS parameter. The SDK
verifies the certificate and hostname, exchanges a session and subscribes to
depth and validity without changing camera settings.

For Docker, mount these files read-only and use their container paths in the
configuration. The supplied image runs as UID 10001, so a host-owned `0600` token
also needs ownership that lets that UID read it. The same-Pi `compose.yaml` mounts the runtime and configuration directories.
Use `compose.remote.yaml` for a remote camera; it mounts credentials and runs
with their owner's numeric UID/GID instead.

DNS and hosts-file names are supported. If the host resolves an mDNS name that
the client cannot resolve, add `connect_address: 192.0.2.10` with the camera's
actual numeric IP. Keep its certificate hostname in `endpoint`; hostname
verification still applies.

HTTPS requires `timestamp_mode: receipt`: message headers contain arrival time
at the adapter, before conversion and publication. Source capture time remains
in `frame_metadata`. There is no remote clock correlation, so these stamps do
not establish acquisition latency or synchronized capture. Remote `cadence_hz`
must be between 1/60 and 60 Hz; the lower limit lets the SDK's deadline cover two
frame intervals.

Do not change only `source` in the default `camera.yaml`: it also pins the local
socket endpoint and `timestamp_mode: local_boottime`. Use the complete source
configuration above.

Optional rectified-image/disparity products remain disabled by default. Confirm
the remote gateway supports every requested product before enabling them: an
unsupported requested product rejects the entire HTTPS subscription, including
otherwise supported depth. Automatic capability fallback is not implemented.

## NVR1 replay

Use the supplied [replay configuration](../nuvo_bringup/config/replay.yaml) as a
starting point, or run directly:

```bash
ros2 run nuvo_camera nuvo_camera_node --ros-args \
  -r __ns:=/nuvo/front \
  -p source:=replay -p recording:=/recordings/capture.nvr \
  -p timestamp_mode:=replay_rebased \
  -p frame_id:=nuvo_front_left_rectified_optical \
  -p replay_rate:=1.0 -p replay_loop:=false
```

The recording must be readable inside the process or container. Compose does
not mount `/recordings` by default. Replay needs no camera connection or token.
The format is NVR1; `.nvr` is the example filename extension.

| Parameter | Default | Behavior |
|---|---|---|
| `recording` | empty | Required NVR1 file path |
| `replay_rate` | `1.0` | Playback speed, greater than 0 and at most 100 |
| `replay_loop` | `false` | Reopen at end of file |
| `publish_clock` | `false` | Publish the playback timeline on `/clock` |

By default, the first source timestamp is rebased to current ROS system time.
Later stamps retain source-time differences; `replay_rate` changes wall-clock
pacing, not those differences. Pacing uses a steady clock. With `lazy: true`,
losing measurement subscribers pauses playback at its file position; returning
subscribers resume without a rewind or catch-up burst. At end of file, images
stop and the node keeps publishing diagnostics.

For simulation, set `publish_clock: true` on the driver and `use_sim_time: true`
on consumers. The driver then reports `timestamp_mode: replay_clock` and starts
its timeline from the first source capture time. Only one `/clock` publisher
should be active. If setting `use_sim_time: true` on the driver itself, also set
`publish_clock: true`. Live sources reject simulated time. `use_sim_time` is fixed
at startup for every source.

A replay loop or new source-time segment continues after the preceding ROS
stamp. Original source times and capture identity remain in metadata. Older
recordings may need the geometry override below.

## Beta 8 and older recordings

CameraInfo and XYZ require the calibration for the actual matching grid. Public
camera beta 8 and some older recordings do not transport that geometry. Depth
and XYZ then need an explicit `calibration_override_file`. Missing matching
geometry blocks those outputs. Rectified images require producer image-grid
geometry, which this matching-only override cannot supply; disparity also needs
the producer's validated range, quantization and ROI declarations.

This adapter has no automatic geometry-override exporter. Prepare the geometry
from the calibration bundle actually used for that capture, accounting for the
matching grid's exact crop and resampling. Raw-image calibration matrices cannot
be copied directly into the override.

For the supplied Compose setup, save the prepared override as
`nuvo_bringup/config/geometry.json` on the host; that directory is mounted at
`/config`. Add this entry under `ros__parameters` in the driver configuration:

```yaml
    calibration_override_file: /config/geometry.json
```

The table defines the required JSON input schema; it does not generate the
geometry values:

| Field | Required value |
|---|---|
| `calibration_sha256` | Exact 64-character calibration hash carried by the frame |
| `matching_grid.width`, `matching_grid.height` | Depth/validity output dimensions |
| `matching.left_projection` | Row-major 3 x 4 calibrated left projection, 12 numbers |
| `matching.right_projection` | Row-major 3 x 4 calibrated right projection, 12 numbers |
| `matching.reprojection` | Row-major 4 x 4 Q matrix, 16 numbers |
| `matching.baseline_m` | Positive calibrated baseline in metres |

The SDK checks the hash, depth dimensions and consistency of the projections,
Q and baseline. It applies an override only when producer geometry is absent.
See the [SDK reference](../nuvo_depth_client_vendor/upstream/README.md) for the
underlying client contract. Do not substitute guessed focal lengths, a nominal
baseline or a shared seed for the unit's active calibration.

Calibration authority is preserved: `0` is provisional, `1` user-validated and
`2` factory-validated. Provisional frames require
`allow_provisional_calibration: true` to publish; an override cannot validate
them. Blocked live sources release acquisition demand and check again every
five seconds. Blocked replay keeps its playback pacing. Existing validated
per-unit bundles remain authoritative in the native runtime; a ROS upgrade does
not require recalibration.

## Identity, calibration and clock policies

These startup parameters apply to every observed capture. Geometry overrides
and identity pins serve separate purposes: an override supplies missing matching
geometry; a pin restricts which device or calibration can publish.

| Parameter | Default | Meaning |
|---|---|---|
| `expected_device_uuid` | empty | Optional nonzero 32-character lowercase hex device identity |
| `expected_calibration_sha256` | empty | Optional nonzero 64-character lowercase hex calibration identity |
| `acquisition_restart_policy` | `accept` | `accept` allows a new live run or advancing capture/configuration epoch; `reject` latches publication off |
| `clock_jump_policy` | `remap` | Local-boottime policy: `remap`, `reject` or `latch` |
| `clock_jump_threshold_s` | `0.1` | Positive finite change in the system-to-boottime offset that counts as a jump |
| `maximum_frame_age_s` | `1.0` | Freshness limit, greater than 0 and at most 30 seconds |

Pins remain empty in the default configuration. Fill the intended identities in
the [strict example](../nuvo_bringup/config/strict.yaml) before deploying it.
A mismatch blocks that capture. A new qualifying capture can recover a pin
mismatch; changing the connected device after continuity is established latches
an identity fault. Existing validated calibration bundles remain authoritative.

Live run/capture/configuration identity is retained across reconnects and lazy
pauses. Duplicate captures, regressing epochs/sequences and regressing source
time are rejected. Sequence resets require an advancing capture epoch or a new
run. Local-boottime capture time must still advance across runs in the same host
boot. Receipt mode allows a new remote run to begin with a reset source clock.

Clock policies apply to local `local_boottime` mapping. `remap` accepts the new
offset; `reject` blocks the jump and can recover on a new valid capture with a
stable clock; `latch` blocks publication until the node restarts. Device-change
and `acquisition_restart_policy: reject` faults also require a node restart.
Investigate and reaffirm the configured identities before restarting. A transport
reconnect alone does not clear a latch. A latched fault releases live acquisition
demand rather than keeping matching active behind blocked output. Replay
segments and loops use their own
timeline handling rather than the live restart policy.

## Validated measurement input

The C++ SDK checks typed producer depth/disparity constraints before ROS
conversion. It rejects unknown validity codes, non-NaN invalid depth, infinite
measurements, producer stereo-configuration errors and accepted samples outside
declared depth/disparity ranges or valid ROI. When both depth and disparity and
matching geometry are present, it verifies their calibrated optical-axis Z
relationship, including the principal-point offset.

Depth-only acquisition still requests only depth and validity. Older recordings
can omit constraint declarations; the SDK cannot check limits or geometry that
were never supplied. Image-only acquisition has no measurement-constraint
prerequisite. These checks validate the transported contract and do not establish
physical distance accuracy.

## Image, cloud and metadata semantics

Depth is optical-axis Z in metres (`32FC1`). Samples with a validity code other
than `1`, or a nonfinite/nonpositive depth, become NaN. `depth/validity` (`8UC1`)
retains the original rejection codes.

Depth, validity and CameraInfo share a timestamp, optical frame and matching
grid. CameraInfo describes the already rectified virtual pinhole camera: P is
the active left projection, K comes from P, R is identity and distortion is
zero. It does not describe the raw distorted sensor image. Supply the mounting
transform in your robot's URDF/TF tree.

`publish_points: true` enables organized XYZ PointCloud2 output in the depth
optical frame. XYZ is computed only while `points` has subscribers. The default
`point_stride: 2` samples every second pixel in each dimension; the accepted
range is 1–16. Invalid XYZ values are NaN.

Optional rectified-image and disparity publishers are disabled by default:

| Startup flag | Topics | Contract |
|---|---|---|
| `publish_left_rectified` | `left/image_rect`, `left/camera_info` | Producer image grid; `mono8`, `rgb8` or `bgr8` without color conversion |
| `publish_right_rectified` | `right/image_rect`, `right/camera_info` | Right image grid and calibrated right projection, including stereo translation |
| `publish_disparity` | `disparity` | `stereo_msgs/msg/DisparityImage` on the matching grid |

The left image, depth and disparity use `frame_id`. Right images use
`right_frame_id`, whose executable default is `nuvo_right_rectified_optical`.
Image CameraInfo comes from actual producer image geometry, never from the
matching-only override or raw sensor intrinsics. Image and matching resolutions
can differ. Associated products retain the same capture timestamp and identity;
their dimensions and optical frames describe their respective grids.

Disparity is the native value minus `(cx_left - cx_right)`, following the ROS
stereo convention. Its `f` and `t` come from matching focal length and calibrated
baseline; min/max use the same offset, `delta_d` preserves native quantization
and `valid_window` preserves native ROI. Disparity publication requests the
matching validity plane and emits NaN for every code other than `1`, including
finite native disparities rejected by a later depth check. It preserves native
NaNs and does not change the SDK's immutable native buffers. Missing required
geometry, validity or producer constraints blocks the requested conversion.
Other usable products from the capture can still publish; diagnostics expose
per-product availability and `product_unavailable_reason`. This conversion
behavior does not provide fallback for an unsupported HTTPS subscription.

[FrameMetadata](../nuvo_camera_msgs/msg/FrameMetadata.msg) accompanies each
published measurement and retains:

- Device/run UUIDs, capture epoch/sequence, configuration epoch and stream sequence.
- Original capture timestamp and clock, plus calibration hash and authority.
- Applied timestamp mode, timestamp uncertainty and replay flag.

For local acquisition, `local_boottime` maps native `CLOCK_BOOTTIME` capture time
to ROS system time with a bracketed clock sample. `timestamp_uncertainty_ns`
describes that local sampling uncertainty, not camera hardware accuracy; it is
`-1` for receipt and replay modes. The daemon and client must share the native
boottime namespace; explicit Docker time offsets are rejected.

Local frames older than `maximum_frame_age_s` (default 1 second) or more than
5 ms in the future are rejected. Clock jumps follow the configured policy above.
`timestamp_mode: receipt` is also available for local acquisition
when arrival stamps are explicitly wanted; the capture-age check applies to
`local_boottime` mode.

Measurement publishers use volatile best-effort QoS with depth 1 by default.
`reliability` accepts `best_effort` or `reliable`; `qos_depth` accepts 1–16. Set
them before startup. CameraInfo is published per frame with the same QoS. Use
the [rosbag2 overrides](../nuvo_bringup/config/recording-qos.yaml) when recording
the default topics, for example:

```bash
ros2 bag record --qos-profile-overrides-path \
  nuvo_bringup/config/recording-qos.yaml \
  /nuvo/front/depth/image_rect /nuvo/front/depth/validity \
  /nuvo/front/depth/camera_info /nuvo/front/frame_metadata
```

Adjust topic names and the override file if you change the namespace. Reliable
DDS cannot recover captures already skipped by the native best-effort source.
`delivery` controls the native subscription separately from ROS `reliability`.
It defaults to `best_effort`. Local IPC accepts `auto` or `best_effort` and
delivers best effort; `exact` is HTTPS-only and is rejected for local acquisition.

## Diagnostics and availability

`/diagnostics` publishes a reliable status every second. Its status name is the
node's fully qualified name. Diagnostics alone do not create acquisition demand
when `lazy: true`. Live transport failures recreate the same source with bounded
retry backoff; local acquisition never falls back to HTTPS.

| Diagnostic values | Interpretation |
|---|---|
| `source_age_s`, `acquisition_age_s`, `source_age_available` | Capture age from local boottime; both ages are `-1` and availability is false for receipt/replay |
| `observation_age_s`, `publication_age_s` | Steady-clock age of the last new observation and successful publication; `-1` before one exists |
| `output_available`, `depth_available`, `left_rectified_available`, `right_rectified_available`, `disparity_available` | Fresh publication availability, with per-product flags |
| `calibration_qualified`, `calibration_sha256`, `calibration_authority` | Native authority and identity; provisional remains unqualified even with preview enabled |
| `depth_minimum_m`, `depth_maximum_m`, `disparity_accepted_min_px`, `disparity_accepted_max_px`, `disparity_quantization_px` | Producer constraints when present; missing declarations are not invented |
| `identity_fault`, `clock_fault`, `fault_latched`, `reject_reason` | Current qualification or policy failure |
| `product_unavailable_reason` | Conversion failures for requested products; inspect each required product's availability even when another product publishes |
| `observed_*`, `published_*` | Latest observed and published device/run/capture identity |
| `published_frames`, `acquisition_errors`, `duplicate_frames`, `stale_frames`, `rejected_frames`, `acquisition_restarts`, `clock_jumps` | Counters for diagnosis |

No-frame waits and duplicates do not refresh successful publication. Availability
expires after `maximum_frame_age_s`, evaluated on the diagnostic timer; consumers
still need their own deadlines. `calibration_qualified` describes native authority,
not whether a configured hash pin matched. A running process, fresh diagnostics
or accepted provisional output does not establish robot health. See the
[integration guide](integration.md) for commissioning and downstream checks.

## DDS transport

The Docker image and Compose setup select Cyclone DDS with
`CYCLONEDDS_URI=file:///ws/install/share/nuvo_bringup/config/cyclonedds.xml`.
The node remains RMW-agnostic: deployments can override `RMW_IMPLEMENTATION`
and supply an independently qualified middleware/transport profile.
The [supplied profile](../nuvo_bringup/config/cyclonedds.xml) sets
`MaxMessageSize` to `1400B` and `FragmentSize` to `1200B`. This lets DDS fragment
large images and clouds while keeping UDP datagrams within an ordinary
1500-byte Ethernet MTU, avoiding dependence on IP reassembly. Interface
selection, discovery and socket buffers retain their defaults.

For a source workspace, set the profile before starting ROS participants, from
the repository root after sourcing the workspace:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$PWD/nuvo_bringup/config/cyclonedds.xml"
```

Use an absolute profile path without spaces. Apply the profile to publishers
and consumers during initial qualification. The host full-size test passed
with automatic interface selection and original `rmem_max`/`wmem_max` values of
212992; a permanent host buffer increase is not part of this fix. Network
discovery, firewalls, effective MTU and transport selection still need checking
for each deployment. This DDS configuration is separate from the local Nuvo
Unix-socket/memfd acquisition path.

An existing robot can retain Fast DDS with its independently qualified profile.
This record does not qualify Fast DDS; no Fast DDS example is supplied.

The [validation record](validation-20261026.md) distinguishes confirmed host and
container results on amd64 and hosted ARM64 from pending Pi hardware checks.

## Source build and development

Use Ubuntu 24.04 with ROS 2 Jazzy ros-base or desktop installed. From this
repository root, install the additional build dependencies used by the Docker
build:

```bash
sudo apt update
sudo apt install -y build-essential cmake ninja-build \
  python3-colcon-common-extensions nlohmann-json3-dev libboost-system-dev \
  libssl-dev libc-ares-dev ros-jazzy-rclcpp-components ros-jazzy-sensor-msgs \
  ros-jazzy-diagnostic-msgs ros-jazzy-stereo-msgs ros-jazzy-rosgraph-msgs \
  ros-jazzy-ament-cmake-gtest ros-jazzy-rmw-cyclonedds-cpp
source /opt/ros/jazzy/setup.bash
MAKEFLAGS=-j2 colcon build --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --return-code-on-test-failure
colcon test-result --verbose
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$PWD/nuvo_bringup/config/cyclonedds.xml"
ros2 launch nuvo_bringup camera.launch.py
```

For an existing colcon workspace, clone this repository under its `src/`
directory and run colcon from the workspace root. The same packages are built;
there is no dependency on a Nuvo robot workspace. See the
[robot integration examples](using-in-robot.md) for launch and remapping.

The component is registered as `nuvo_camera::NuvoCamera`; the standalone
executable is `nuvo_camera_node`. Driver parameters are startup-only. Connecting
registers product demand without changing exposure, stereo, calibration or
encoder settings.

A C++ package that instantiates the node or uses the public conversion helpers
can declare `<depend>nuvo_camera</depend>` and link the installed target:

```cmake
find_package(nuvo_camera REQUIRED)
add_executable(my_camera_app src/main.cpp)
target_link_libraries(my_camera_app PRIVATE nuvo_camera::nuvo_camera_component)
```

The target supplies its headers, C++20 requirement and transitive dependencies.
Ordinary topic consumers need only the standard message packages they use;
`nuvo_camera_msgs` is needed only for capture metadata.

For a local/replay-only source build, omit `libboost-system-dev`, `libssl-dev`
and `libc-ares-dev` from the dependency command and add
`-DNUVO_CLIENT_WITH_HTTPS=OFF` to the CMake arguments.
For Docker use `sudo docker compose --env-file docker/.env -f docker/compose.yaml build --build-arg NUVO_CLIENT_WITH_HTTPS=OFF`.
The Docker flag omits explicit HTTPS dependency requests from both build and
runtime stages and removes the client's direct HTTPS library links. The ROS base
image can still contain TLS libraries through its own dependencies.
Docker's compiler parallelism defaults to two jobs and can be changed with
`--build-arg BUILD_JOBS=4`.

The generated SDK snapshot lives in `nuvo_depth_client_vendor/upstream`. Make
acquisition, decoding and NVR1 fixes in Nuvo Depth's canonical `sdk/cpp`, then
refresh from that repository:

```bash
python3 tools/refresh_sdk.py --source-repo "/path/to/Nuvo Depth"
python3 tools/refresh_sdk.py --check
```

Do not hand-edit the snapshot. `SOURCE-MANIFEST.json` records the source commit,
file hashes and license notices; `--check` verifies it without a sibling source
checkout. Builds use the checked-in snapshot without downloading an SDK branch.
Nuvo-authored code and the SDK use GPL-3.0-or-later.

After building, sourcing the workspace and setting the DDS profile above, the
repository's additional installed-package and DDS checks can be run with:

```bash
python3 tools/refresh_sdk.py --check
python3 test/installed_consumer.py
python3 test/robot_integration.py
python3 test/integration.py
python3 test/fullsize.py
```

These use synthetic frames, local socket fixtures and NVR1 replay. They do not
connect to a physical camera. The [generic-driver audit](audit-20261026.md)
records the tested boundary and remaining hardware/native-version requirements.
