# Startup controls for robot prototyping

These controls are included in ROS **v0.1.3** / `jazzy-0.1.3` and native
**v0.1.1-beta.9**. The native minimal and full packages share the camera pipeline
and local control implementation. Native beta 8 lacks calibration control and
retains its [geometry-override requirement](reference.md#beta-8-and-older-recordings).

## Run on the camera's Pi

Install native Nuvo Depth using the minimal installer for a new minimal system,
or upgrade within the profile already installed. Current native installation
prepares the camera service, stable runtime directory, `nuvo-video` permissions,
matching-grid geometry and local calibration control socket automatically.
No gateway, ROS installation on Pi OS, camera-device mounts or privileged
container are needed. Camera overlays can still require the installer's reboot.
Docker Engine/Compose must be installed on the host.

Use the [no-build bootstrap](../README.md#quick-start) on the Pi:

```bash
curl -fsSL https://raw.githubusercontent.com/arputhamips/nuvo-depth-ros/v0.1.3/install-nuvo-depth-ros.sh \
  | sudo bash -s -- --outputs depth,points --cadence-hz 10
```

The bootstrap pins the image digest and deployment source revision, retains
parameters in `/etc/nuvo-depth-ros/camera.yaml` and invokes `tools/run_camera.py`.
The helper detects the numeric native group ID, checks native health and starts
the existing Compose service. It does not create or overwrite `.env` or your
existing parameter YAML. `--namespace robot/front`, `--ros-domain-id 42` and
`--config /path/to/camera.yaml` are optional. Without an explicit output/rate
argument, the selected YAML remains authoritative. `--dry-run` prints the
resolved deployment without starting a container.

The same helper prepares a host DDS receive-buffer minimum of 8 MiB by raising
only `net.core.rmem_max` when needed. It installs the root-owned
`nuvo-depth-ros-network.service`, ordered after normal sysctl setup and before
Docker at boot. Higher limits and default socket buffers are preserved. The
helper recreates the ROS container on every explicit launch so DDS opens new
sockets, including retries after a failed start. Run it with `sudo`; it reports
setup errors before starting Compose. `--dry-run` reports this host plan on
stderr while keeping the resolved Compose JSON on stdout, and does not write a
unit or change the kernel limit. The bundled Cyclone DDS profile requests an
8 MiB receive buffer to use the prepared host limit. Consumers also need the
profile and a suitable receive limit on their own host; the camera container
loads the profile automatically. See [DDS transport](reference.md#dds-transport).

For a fresh unit using the shared provisional seed, explicitly add
`--allow-provisional-calibration` for inspection. This permits publication; it
does not establish distance accuracy. Existing validated per-unit calibration
remains selected when no calibration argument is supplied. An intentionally
disabled capture policy remains disabled unless native setup or an explicit
accepted calibration import enables it.

With a v0.1.3 source checkout, you can invoke the helper directly instead:

```bash
sudo python3 tools/run_camera.py --outputs depth,points --cadence-hz 10
```

It defaults to the prebuilt release image. `--image` can select an explicitly
tested development image; the helper rejects images without startup-control
support. See the [Docker guide](docker-image.md) for optional source builds.

## Launch directly in ROS

In an environment with v0.1.3 installed:

```bash
ros2 launch nuvo_bringup camera.launch.py \
  namespace:=robot/front cadence_hz:=10 \
  publish_depth:=true publish_points:=true \
  publish_left_rectified:=false publish_right_rectified:=false publish_disparity:=false
```

| Argument / ROS parameter | Meaning |
| --- | --- |
| `cadence_hz` | Requested delivery ceiling, 0.001–60 Hz locally; actual throughput depends on native capture/matching and selected outputs. |
| `publish_depth` | Depth image, validity and matching CameraInfo; defaults to true. |
| `publish_points` | Organized XYZ; conversion runs only with cloud subscribers. |
| `publish_left_rectified`, `publish_right_rectified` | Rectified images and their CameraInfo. |
| `publish_disparity` | Calibrated matching-grid disparity. |
| `calibration_file` | Explicit native stereo bundle activation, local source only; empty retains native selection. |
| `allow_provisional_calibration` | Explicit preview opt-in; authority remains provisional. |

Optional launch arguments have empty defaults so they do not override values in
`config:=/path/to/parameters.yaml`. Explicit arguments take precedence. Both
`camera.launch.py` and `inspection.launch.py` expose the same controls. Direct
node/component users set the same typed ROS parameters. They are startup
parameters: relaunch to change them.

Select image-only output without running the native depth matcher:

```bash
ros2 launch nuvo_bringup camera.launch.py \
  publish_depth:=false publish_left_rectified:=true cadence_hz:=15
```

The default lazy policy requests native products only when enabled measurement
topics have subscribers. Disabling depth hides depth/validity/CameraInfo topics;
points can still request depth internally. A metadata subscriber can request
the enabled products, but diagnostics alone do not. The requested cadence does
not reprogram camera triggering or promise that a slow matcher will meet it.
Use `nuvoctl` over SSH for capture, exposure, matching and other device settings.

## Select an existing calibration bundle

Pass a complete compatible native stereo bundle: an extracted directory, or an
uncompressed `.tar` with `manifest.json`, `front_left.yaml`, `front_right.yaml`
and `validation.json` at its root. A single ROS CameraInfo YAML or geometry-only
JSON is not a native calibration bundle. Use the original per-unit bundle and
preserve its validation, hashes and signatures.

```bash
sudo bash install-nuvo-depth-ros.sh \
  --calibration-file /home/pi/calibration/front.tar \
  --outputs depth,points --cadence-hz 10
```

The helper stores a bounded snapshot under `/var/cache/nuvo-depth-ros/inputs/` and
mounts only that snapshot read-only. This lets the unprivileged container read
an input originally saved with mode `0600` and restart later without changing
your original file permissions. Relaunch the helper after changing an input;
keep its snapshot while that container exists. A direct ROS launch uses the path
visible in its process/container:

```bash
ros2 launch nuvo_bringup camera.launch.py calibration_file:=/calibration/front.tar
```

The saved installer above comes from the [versioned download](../README.md#quick-start).
For provisional inspection, explicitly add `--allow-provisional-calibration`;
selecting a bundle does not implicitly enable provisional publication.

Native Nuvo Depth validates the bundle, performs its existing activation and
rollback transaction, and verifies fresh capture before returning success. ROS
waits before acquiring measurements and checks the returned calibration identity
on subsequent frames. Selecting the already-active bundle verifies it without
an unnecessary restart. The default empty argument makes no control request.

Calibration selection belongs to the camera and persists across ROS restarts;
it affects every client of that camera. An explicit different bundle can restart
native capture. Applications sharing a camera should agree on one selection.
The native control socket grants this operation to the existing `nuvo-video`
group and authenticates the privileged native service. ROS receives no host
configuration write mount or general command execution interface.

`calibration_file` rejects HTTPS/replay sources. For a remote camera, import the
bundle with native `nuvoctl calibration import` over SSH, then launch the driver.
`calibration_override_file` remains a separate compatibility setting for older
streams missing matching geometry; it does not activate or change native
calibration.
