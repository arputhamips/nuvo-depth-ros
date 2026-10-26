# Development validation — 25 October 2026

Implementation target: native Nuvo Depth on Raspberry Pi OS Lite 64-bit, with
ROS 2 Jazzy in an Ubuntu Docker container on the same Raspberry Pi 5.
This record is host/container verification, not Pi hardware qualification.

The canonical SDK lives in the sibling Nuvo Depth repository under `sdk/cpp`.
Its generated snapshot records base commit, exact LF-normalized source hashes,
licenses and notices in `nuvo_depth_client_vendor/upstream/SOURCE-MANIFEST.json`.

SDK content SHA-256:
`1c09096265578bbdb603bbc63e381d77633c619df80f965c79c75f45820c9de5`.

## Verified boundaries

| Check | Result |
|---|---|
| Existing Nuvo Depth minimal-host regression suite | 52/52 passed |
| Standalone exported C++ SDK with HTTPS | 3/3 CTests passed |
| Standalone C++ SDK without HTTPS dependencies | 3/3 CTests passed |
| External consumer using only installed headers/CMake target | Passed |
| Actual native NUV1 serializer → C++ decoder → Python NVR1 → C++ replay | Passed |
| ROS conversion, projection and parameter tests | 11 gtest cases passed; colcon reports 13 including wrappers |
| Docker multistage build and colcon tests | Passed |
| Real ROS/DDS replay, source-relative simulation clock and XYZ | Passed with Cyclone DDS |
| Native host socket → unprivileged ROS container → host DDS | Passed |
| Numeric supplementary GID and read-only runtime-directory mount | Passed |
| Native host/container CLOCK_BOOTTIME mapping and freshness | Passed |
| Host memory handoff with container `--network none --ipc private` | Passed; ROS publication observed without external container networking |
| ROS native demand excludes encoded preview | Passed; only depth + validity at 10 Hz, preview demand zero |
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

The reconnect test replaces the socket inode within a retained runtime
directory. It does not qualify systemd `RuntimeDirectory` deletion and
recreation. After an actual native service restart, wait for native capture to
be active and [recreate the camera container](reference.md#local-only-operation)
to refresh its runtime-directory bind mount.

## Environment and limits

Validation ran on Linux amd64 in Ubuntu 24.04 under WSL2 and an Ubuntu Jazzy
Docker image. The final unprivileged runtime image was:

`sha256:c21f96e6f44bab3a3196759b5f5ce308a2c693138b9dcb80d327c18cd26ab16e`.

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

## Integration audit follow-up

Three GPT-6.1 Sol agents with extra-high reasoning audited the node, canonical
SDK and deployment boundary. Changes were reviewed by the existing chats
“Research Nuvo Depth for ROS” and “Research first-time setup flow.” Both accepted
the source changes; the core acceptance names the exact SDK hash above.

- Removed Nuvo Depth's 331-line `tools/pi/validate_ros_bridge.py`, which checked
  retired topics, ring diagnostics and a hardcoded 161.46 mm rig baseline. The
  Pi-tools README now points to this adapter's current integration tests.
  Parsed and tested configuration compatibility fields remain supported.
- HTTPS frames retain their contiguous receive buffer, avoiding an extra full
  payload allocation and copy. IPC mappings release received descriptors
  immediately. Tests retain two frames across later acquisition, cancellation
  and source destruction, verify unchanged bytes and count retained descriptors.
- Malformed/partial acquisition is terminal; ordinary empty IPC waits remain
  reusable and invalidation remains distinguishable. Replay checks declared
  lengths against the opened file's finite snapshot before allocation.
- Geometry rejects nonfinite derived matrices, zero/negative encoded baselines
  and mismatched focal lengths or small baselines, including explicit overrides.
  Calibration identity, authority and the existing per-unit/seed policy are intact.
- Lazy replay preserves its cursor and pauses pacing. Deduplication follows
  successful publication, receipt stamps precede conversion, invalid ROS time
  ranges fail before a replay wait, and runtime clock-policy changes are rejected.
  HTTPS cadences below 1/60 Hz fail at startup to honor the 120-second deadline.
- Rejected provisional replay still follows playback pacing. A blocked live
  source closes its subscription and probes every five seconds, releasing native
  matcher demand between probes and recovering when validated calibration appears.
- XYZ conversion validates geometry and buffer bounds, respects padded rows and
  precomputes row/column projection factors instead of dividing for every point.
  XYZ remains optional and subscriber-driven. No Pi speedup is claimed from this
  host validation.
- Snapshot checks validate aggregate provenance and reject corruption under both
  normal Python and `python -O`. Six new snapshot regression tests pass.
- Deployment CI now builds the final image and exercises a native host fixture
  through the unprivileged runtime. It records resolved package/image versions.
  Both CI and Docker use the same pinned base; Docker compiler parallelism
  defaults to two jobs through `BUILD_JOBS`.

Final clean Release SDK builds passed all three CTests with HTTPS enabled and
disabled, including installed public consumers. The native serializer, C++
decoder, Python NVR writer and C++ replay interoperability check passed. The
fresh Nuvo Depth minimal-host suite passed 52/52 tests; setup consistency and
CI routing also passed.

The expanded DDS suite passed replay loops and pauses, timestamp policies,
future-frame rejection followed by acceptance of the same identity, duplicate
suppression, provisional-calibration pacing/demand release/recovery and restart.
The ROS reviewer's remaining clock-coverage request was implemented and passed:
the test subscribes to actual `/clock` messages, checks their equality to frame
stamps and monotonic source-relative loop values, and checks silence when clock
publication is disabled.
`test/native_recording.py` also passed using the actual native serializer's
unchanged frame bytes, checking calibration, identity, depth and projected XYZ.

The final Docker build passed its colcon checks. The host/container test passed
with runtime UID 10001 and the host fixture's explicit supplementary GID. It
first proved permission denial without that group, then checked both native run
identities across socket replacement and clean SIGINT exit code 0. Local logs,
XML results, image identity and package inventories are under `output/audit/`.
The new hosted workflow is configured; no GitHub run or release was created by
this audit. ARM64 and real-camera qualification remain outstanding.

## Same-device local-only follow-up

The existing final runtime image also passed acquisition with Docker networking
disabled and a private IPC namespace. With only the read-only runtime directory
and numeric group supplied, UID 10001 made one local connection, requested six
frames, reached actual ROS publication and shut down with exit code 0. Its exact
native subscription selected depth and validity (mask 40), cadence 10000 mHz and
encoded-preview cadence zero. The host observed the source connection close.
The existing host-network DDS/restart/permission tests passed in the same run.
Evidence is in `output/audit/deployment-no-network/`, including immutable image
identity in both contract reports. This tests host/container transport, not Pi
camera performance or actual DDS latency under load.

Both native service profiles restrict the daemon to `AF_UNIX`; the network
gateway is a separate process. The minimal profile omits it and the encoder.
The ROS README describes keeping an existing full install/calibration intact
while masking the gateway for explicit local-only operation. The host network
remains available to ROS DDS and SSH. No Pi services were changed by these
host-side tests and documentation edits.

A clean current-source native gateway/encoder Release build also passed six
targeted gateway demand checks and the local-frame-server/encoder CTests (2/2).
The checks cover idle gateway requests, subscription union, cadence separation,
disconnect release and bounded snapshot acquisition. The targeted harness kept
the existing test order and published a fresh fixture frame before the existing
class shutdown check. No runtime source changes were required. An idle gateway
does not register raw or encoded native demand; the native encoder receives
zero preview demand for the ROS-only subscription. Explicit browser/remote
subscriptions or diagnostic trace/snapshot flags remain additional consumers.
