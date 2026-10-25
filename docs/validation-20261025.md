# Development validation — 25 October 2026

Implementation target: native Nuvo Depth on Raspberry Pi OS Lite 64-bit, with
ROS 2 Jazzy in an Ubuntu Docker container on the same Raspberry Pi 5.
This record is host/container verification, not Pi hardware qualification.

The canonical SDK lives in the sibling Nuvo Depth repository under `sdk/cpp`.
Its generated snapshot records base commit, exact LF-normalized source hashes,
licenses and notices in `nuvo_depth_client_vendor/upstream/SOURCE-MANIFEST.json`.

SDK content SHA-256:
`12b8177c746bd70e39a583faf386ff3316d6dcaaff774de5cd8e252ede5b4cb6`.

## Verified boundaries

| Check | Result |
|---|---|
| Existing Nuvo Depth minimal-host regression suite | 52/52 passed |
| Standalone exported C++ SDK with HTTPS | 3/3 CTests passed |
| Standalone C++ SDK without HTTPS dependencies | 3/3 CTests passed |
| External consumer using only installed headers/CMake target | Passed |
| Actual native NUV1 serializer → C++ decoder → Python NVR1 → C++ replay | Passed |
| ROS conversion and projection tests | Passed |
| Docker multistage build and colcon tests | Passed |
| Real ROS/DDS replay, source-relative simulation clock and XYZ | Passed with Cyclone DDS |
| Native host socket → unprivileged ROS container → host DDS | Passed |
| Numeric supplementary GID and read-only runtime-directory mount | Passed |
| Native host/container CLOCK_BOOTTIME mapping and freshness | Passed |
| Native socket inode replacement, run identity change and reconnect | Passed |
| Actual `ros2 launch` with namespace and YAML configuration | Passed |
| Git-staged SDK checksums across Windows/Linux line endings | Passed |

The transport tests exercise malformed frame/record bounds, overlaps and masks,
invalid geometry, pixel-grid consistency and reference extents, calibration
fallback identity/grid checks, preservation of provisional authority, sealed
SCM_RIGHTS ownership, invalidation, full socket backlog, FIFO rejection,
cancellation, repeated EINTR, trusted and untrusted TLS, hostname rejection,
numeric routing with unchanged hostname verification, closing/persistent HTTP,
session cleanup and stalled asynchronous DNS deadlines.

The numeric fixtures are small synthetic images with known projections. Native
serialization and recording interoperability use the actual repository code.
The host/container integration uses a native socket fixture rather than cameras;
it checks the OS/container/DDS boundary and numeric conversion, not scene depth
quality or physical sensor timing.

## Environment and limits

Validation ran on Linux amd64 in Ubuntu 24.04 under WSL2 and an Ubuntu Jazzy
Docker image. The final unprivileged runtime image was:

`sha256:438983c1f7c6a4b3482c7580ff767d72b8726fc1f31fcc31057dd67c40ebe317`.

Its pinned base manifest is
`ros:jazzy-ros-base@sha256:103b6c417973394659af6804d4917cd489b91e5ed22eb7bef263025a3bed2410`;
the registry manifest contains amd64 and arm64/v8 variants. The ARM64 image was
not built or tested in this run. No ARM64 package/signing workflow was started.

Cyclone DDS passed both same-container and native-host/container checks and is
the default. Fast DDS failed discovery on this WSL host, including the container
test. It is not qualified by this record. This did not indicate a conversion or
point-cloud math defect; the same contracts pass with Cyclone DDS.

Before claiming Pi support, run native arm64 build/install and real-camera
acceptance on the pinned camera cohort: accepted/provisional calibration policy,
beta 8 explicit geometry fallback, sustained rate/latency/resources, clock jumps,
restarts, long acquisition/session expiry, shutdown, namespaces, RViz, rosbag2
QoS/replay and the intended robot DDS configuration. Do not infer metric
accuracy or smaller-memory Pi qualification from host fixtures.

## Setup-flow review

The user-requested reviewer, “Research first-time setup flow,” reviewed all Nuvo
Depth additions and the small CI/native-fixture edits. Its findings were applied:
bounded socket connection/wait deadlines, explicit invalidation, regular-file
replay, cross-grid geometry consistency/reference bounds, HTTP closure handling,
interruptible DNS without background threads, safe callback lifetime, an example
that survives ordinary empty waits, and a maintained EINTR regression.
The reviewer approved the SDK structure after those corrections. HTTPS cleanup
also uses a fresh bounded connection after a long stream, rather than assuming
an idle HTTP connection remains usable.

No native capture/matching/calibration, installer/package dependency, service or
device configuration was changed by this implementation. Existing unrelated
documentation edits in Nuvo Depth were preserved. No tags, releases or remote
repository publication were created.
