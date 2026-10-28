# Changelog

## 0.1.3 — 2026-10-28

- Prepare the same-Pi host for DDS image delivery by raising only
  `net.core.rmem_max` to at least 8 MiB, preserving higher values. The shared
  launcher installs a conditional boot service after normal sysctl setup and
  before Docker, so restarts retain the minimum without replacing higher
  administrator settings. Default receive/send buffers stay unchanged.
- Request an 8 MiB receive buffer in the supplied Cyclone DDS profile for
  image/cloud bursts. The minimum remains `default`, allowing startup on hosts
  with smaller caps. Consumers need the same profile and suitable host limit;
  the camera container loads the profile automatically.
- Recreate the ROS container on every explicit helper launch so DDS reopens its
  sockets, including retries after an interrupted setup. Host setup failures
  stop before container startup; helper `--dry-run` reports the plan without
  changing host configuration.
- Keep the public bootstrap delegated to the same helper and publish the patch
  as `jazzy-0.1.3`. Native `v0.1.1-beta.9`, calibration ownership and the
  unprivileged container deployment are unchanged. See the
  [validation record](docs/validation-20261028.md) for bounded Pi evidence;
  host/container tests alone do not establish hardware qualification.

## 0.1.2 — 2026-10-28

- Expose typed output, requested cadence and calibration-bundle launch arguments,
  retaining YAML settings when an argument is omitted. Depth topics can be
  disabled independently of point clouds or rectified-image outputs.
- Delegate explicit same-device calibration selection to native validation and
  activation, then verify returned identity before publishing. Ordinary startup
  preserves the camera's existing calibration and settings.
- Add a same-device Docker helper that discovers the native numeric group and
  mounts explicit configuration/calibration read-only. Compatible native builds
  install the required socket and permissions automatically.
- Publish the controls in `jazzy-0.1.2` for Linux AMD64 and ARM64. Native
  `v0.1.1-beta.9` supplies matching-grid geometry, runtime permissions and local
  calibration control in both full and minimal profiles. Older native beta 8
  retains its geometry-override requirement and lacks bundle activation.
- Add a no-build GitHub bootstrap that pulls the fixed release, obtains matching
  deployment files from the image's source revision and runs by immutable image
  digest through the existing helper. It discovers the numeric group without
  manual `.env` setup, preserves user YAML across reruns and keeps calibration
  selection empty/provisional publication disabled unless explicitly requested.
- Host/container tests verify deployment contracts; they do not establish Pi
  hardware qualification, sustained throughput or metric accuracy. See
  [startup controls](docs/prototyping.md) for use and native compatibility.

## 0.1.1 — 2026-10-27

- Treat subscriber disconnection during an in-flight frame as a demand pause,
  with unavailable output and a waiting diagnostic. Preserve identity watermarks,
  calibration authority, rejection history and latched faults across pauses.
- Add a read-only native startup check and document the coherent PIO/PWM native
  upgrade required when a legacy GPIO daemon meets a provisioned PWM channel.
- Publish the fixes in the versioned `jazzy-0.1.1` Docker image for Linux AMD64
  and ARM64. Both Compose examples and the quick start use the prebuilt release;
  building from source is optional.
- Record the real Pi 5 retest: 300 coherent frame sets, optional products,
  subscriber pause/resume and recovery after native service restart passed.
  Calibration remained provisional; native startup requires a compatible native
  installation and is not repaired by updating the ROS image alone.

## 0.1.0 — 2026-10-26

Initial ROS 2 Jazzy release for Nuvo Depth. The adapter has no dependency on
Nuvo Base or a robot control/navigation stack. Native Nuvo Depth continues to
own camera configuration, capture, matching and calibration.

### Included

- Standard depth, validity and CameraInfo topics, with optional subscriber-driven
  XYZ, rectified stereo images and calibrated disparity.
- Capture identity, calibration hash/authority and source timing in optional
  metadata, plus ROS diagnostics and configurable acquisition policies.
- Same-host Unix socket/shared-memory acquisition, authenticated remote HTTPS /
  WebSocket acquisition, and NVR1 recording replay.
- Standalone and composable nodes, configurable names/namespaces, frame IDs,
  topic remapping and QoS, and exported C++ component dependencies.
- Ubuntu 24.04 / Jazzy Docker images for Linux AMD64 and ARM64, including an
  unprivileged same-Pi deployment and a remote-camera Compose configuration.
- Beginner setup, multi-camera integration, Ethernet/USB Ethernet connection
  guides, and automated SDK, ROS, DDS and container validation.

### Compatibility and limits

- ROS 2 Jazzy on Linux is the tested ROS environment. No ROS APT package is
  provided. ROS 1, other ROS distributions and Docker Desktop networking are
  not qualified.
- Native Nuvo Depth must already work on the camera host. Public camera beta 8
  needs the documented calibration-hash/grid-matched geometry override.
  Provisional calibration requires explicit opt-in; installing ROS does not
  require recalibration or change calibration authority.
- Remote acquisition requires the native gateway, certificate and token. USB-C
  requires native USB Ethernet provisioning and a working network connection;
  it is not a raw USB/UVC camera interface.
- Hosted AMD64/ARM64 tests validate software contracts and synthetic frame
  delivery. Physical Pi camera operation, sustained performance, metric accuracy,
  and USB-C cable/host/power combinations still require hardware qualification.

Use the [Docker guide](docs/docker-image.md) for the versioned image and
[robot integration guide](docs/using-in-robot.md) for topics, frames and examples.
