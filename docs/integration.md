# Integrating and commissioning the driver

Use this guide when replacing an existing camera adapter or connecting a new
consumer. The [README](../README.md) covers installation; the
[reference](reference.md) defines source, timestamp and measurement contracts.
Keep robot mounting transforms and application decisions in the downstream
integration.

## Pin a compatible deployment

Record the approved driver commit, native runtime revision/package, Docker image
identity, SDK manifest and calibration identity together. Build from that driver
commit rather than an unrecorded moving checkout:

```bash
git clone https://github.com/arputhamips/nuvo-depth-ros.git
cd nuvo-depth-ros
DRIVER_REVISION=REPLACE_WITH_APPROVED_COMMIT
git checkout --detach "$DRIVER_REVISION"
```

Replace the placeholder with the reviewed commit. Follow the README to create
`docker/.env`, build the image and supply the host's numeric `nuvo-video` GID.

Installing the adapter does not update the native runtime. There is no released
minimum native version covering all these changes yet. Use a native revision
with the required capabilities and record the tested revision from the
[validation record](validation-20261026.md):

- Depth/XYZ need matching-grid geometry. Public beta 8 needs the explicit
  hash/grid-matched [geometry override](reference.md#beta-8-and-older-recordings).
- Rectified images need producer image-grid geometry; the matching-only override
  cannot create it. Disparity also needs producer range, quantization and ROI.
- Updated full/minimal native service units preserve `/run/nuvo-depth` across
  restarts and clean generated runtime artifacts. After upgrading older units,
  or replacing the directory manually, recreate the container after native capture
  is active, as described under [local IPC](reference.md#local-only-operation).

Preserve validated per-unit calibration. The provisional shared seed stays
provisional; a software upgrade or driver migration does not require recalibration.

## Map the consumer contract

Inventory the old adapter's topic names, types, units, frame IDs, QoS and time
policy. Update each consumer explicitly rather than retaining assumptions from
the old publisher.

| Consumer input | Nuvo contract |
|---|---|
| Depth | `depth/image_rect`: `sensor_msgs/msg/Image`, `32FC1`, optical-axis Z metres; invalid NaN |
| Depth mask | `depth/validity`: `8UC1`, only code `1` accepted |
| Intrinsics | `depth/camera_info`: actual matching-grid rectified camera, per capture |
| Capture identity | `frame_metadata`: device/run UUIDs, capture/configuration epochs, capture sequence, calibration and source time |
| Optional cloud | `points`: organized XYZ metres in the depth optical frame |
| Optional stereo | `left`/`right` image and CameraInfo topics use the image grid; `disparity` uses the matching grid |
| Driver status | Reliable `/diagnostics`, identified by the node's fully qualified name |

The default measurement namespace is `/nuvo/front`. Choose your deployment's
namespace with the `namespace` launch argument. Set `frame_id` and
`right_frame_id` to the intended optical frames and provide their transforms
through the downstream TF tree. A namespace change does not rename a frame ID.

Measurements default to best-effort, volatile QoS with depth 1. Match subscriber
QoS, and adapt the [recording overrides](../nuvo_bringup/config/recording-qos.yaml)
when changing namespaces. Reliable DDS does not recover skipped native captures.
For Cyclone DDS, use the supplied [transport profile](reference.md#dds-transport)
for publishers and consumers when qualifying large images/clouds. Docker uses it
by default; source workspaces set `CYCLONEDDS_URI` explicitly. The node remains
RMW-agnostic; an existing integration can retain an independently qualified Fast
DDS configuration by overriding `RMW_IMPLEMENTATION` and supplying its profile.
With `lazy: true`, a measurement subscriber creates native demand; diagnostics
and topic discovery alone do not. Optional product flags create publishers, but
those products are acquired only while their outputs have subscribers.

## Configure identities and failure policies

Copy [strict.yaml](../nuvo_bringup/config/strict.yaml) to a deployment-specific
file in the mounted configuration directory. Fill both pins from the intended
unit and its approved active calibration. For example:

```yaml
/**:
  ros__parameters:
    source: local
    endpoint: /run/nuvo-depth/data.sock
    timestamp_mode: local_boottime
    frame_id: camera_left_rectified_optical
    right_frame_id: camera_right_rectified_optical
    expected_device_uuid: REPLACE_WITH_32_LOWERCASE_HEX
    expected_calibration_sha256: REPLACE_WITH_64_LOWERCASE_HEX
    allow_provisional_calibration: false
    acquisition_restart_policy: reject
    clock_jump_policy: latch
    clock_jump_threshold_s: 0.1
    maximum_frame_age_s: 1.0
    cadence_hz: 10.0
    publish_points: false
    lazy: true
```

The placeholder pins deliberately fail validation until replaced. UUID and hash
must be nonzero lowercase hex of the stated lengths. A hash pin remains necessary
when desired even if the producer already supplies geometry; it is independent
of `calibration_override_file`.

In a sourced Jazzy workspace, launch the saved file with your chosen namespace:

```bash
ros2 launch nuvo_bringup camera.launch.py \
  namespace:=sensors/depth config:=/path/to/commissioned.yaml
```

For the supplied Compose deployment, save it under `nuvo_bringup/config`, use
its `/config/...` path and chosen namespace in the service's launch command,
then recreate the service with the README's explicit environment-file options.
Driver parameters are fixed at startup.

Strict restart and clock latches require a node restart after investigation.
Reconnecting or briefly losing subscribers does not reset qualification state.
If accepting acquisition restarts or remapping clock jumps is appropriate, set
those policies explicitly and document that choice. Full behavior is in the
[policy reference](reference.md#identity-calibration-and-clock-policies).

## Commission measurement availability

Before changing downstream consumers, verify this sequence in an isolated test
environment, then repeat the applicable checks on the actual hardware:

1. Subscribe with matching QoS and compare image dimensions, CameraInfo, frame
   IDs and metadata for the same capture. Confirm the configured device/hash,
   native calibration authority and advancing capture identity.
2. Check local source time against `source_age_s` and publication time against
   `publication_age_s`. Receipt and replay source age is unavailable, not zero.
   HTTPS receipt timestamps do not establish synchronized acquisition time.
3. Exercise loss of new captures and duplicate delivery. Availability must expire
   at the configured freshness limit; a fresh diagnostic heartbeat must not
   substitute for a fresh measurement.
4. Exercise native disconnect/restart and subscriber pause/resume. Confirm the
   chosen restart policy and any required container recreation. Test device/hash
   mismatches and clock policies using controlled fixtures before live use.
5. Subscribe to each enabled optional product. Verify its own geometry, encoding
   and frame, including right projection translation and disparity limits/ROI.
   Keep products disabled when the consumer does not need them.
6. Record effective configuration, topic/QoS mapping, package/image versions,
   calibration identities, restart behavior and observed hardware results.

Driver diagnostics describe measurements. `calibration_qualified` reflects native
validated authority and stays false for provisional output; it does not prove
that an expected hash pin matched. Check `reject_reason`, fault flags and fresh
availability for the required products as well. `/diagnostics` updates once per
second, so downstream consumers need their own deadlines. Process liveness,
receipt of a message or a qualified calibration alone does not establish robot
health or authorize application behavior.

The source validates declared measurement ranges, ROI and validity semantics.
That contract check does not establish scene coverage, physical distance
accuracy or sustained hardware performance. Commission those separately.

## Isolated inspection

Use the supplied inspection launch for deliberate provisional evaluation:

```bash
ros2 launch nuvo_bringup inspection.launch.py
```

Its default namespace is `/nuvo/inspection`. The supplied configuration opts
into provisional calibration and enables rectified left/right and disparity
publishers; acquisition still needs subscribers and compatible native metadata.
It remaps diagnostics to `/nuvo/inspection/diagnostics` and any clock output to
`/nuvo/inspection/clock`. The default local source does not publish a clock.

Keep inspection consumers and outputs separate from the commissioned namespace.
Changing preview acceptance does not validate calibration, and a geometry
override does not raise authority. Use a separate strict configuration for
validated measurement consumers; do not promote an inspection namespace by
renaming topics alone.
