# Pi 5 camera validation — 27 October 2026

The idle diagnostic fix and native GPIO/PWM startup recovery passed on an actual
Pi 5 with the dual-IMX296 Nuvo Depth camera. The ROS image was built on that Pi
using the repository's Dockerfile and Compose build command, then run unprivileged
against the installed native systemd service. This test used provisional
calibration with explicit preview permission; it does not establish physical
depth accuracy, electrical trigger timing or long-duration qualification.

## Tested software and hardware

- ROS code: [`c41ca231c822edd7d795bc0a321a0258fbfc64c4`](https://github.com/arputhamips/nuvo-depth-ros/commit/c41ca231c822edd7d795bc0a321a0258fbfc64c4).
- Locally built ARM64 image: `sha256:830f86a8aadeba2d944b184c76e76c66608539a62bafc3aa16de11a6e99cd129`.
  The v0.1.0 registry image was not replaced.
- Native source: [`98fcb4c29bcefd5398f6d2ddc669224917a40f31`](https://github.com/arputhamips/nuvo-depth/commit/98fcb4c29bcefd5398f6d2ddc669224917a40f31),
  complete appliance package cohort from [manual ARM64 build 40525027089](https://github.com/arputhamips/nuvo-depth/actions/runs/40525027089).
  These are development artifacts, not a new native release.
- Pi 5 Model B Rev 1.1, 16 GB; Pi OS/Debian Trixie aarch64;
  kernel `6.18.33+rpt-rpi-2712`; Docker 26.1.5 and Compose 2.26.1.
- The existing 728 x 544, 60 Hz native capture profile, EGL rectification,
  matcher settings and exposure controls were preserved. Requested ROS cadence
  was 10 Hz. Capture cadence is not the matcher output rate.
- ROS domain 93, namespace `/nuvo_inspection/pi5`, UID 10001, supplemental
  `nuvo-video` GID 984, read-only `/run/nuvo-depth` mount; no privileged mode or
  camera-device mounts in the ROS container.

## Results

| Check | Result |
| --- | --- |
| Docker build on the Pi | Passed, including 35 colcon tests |
| Default provisional-calibration gate | Passed: no measurement publication during the 12-second check |
| Depth, validity, CameraInfo, metadata and XYZ | Passed: 83 coherent frame sets checked in 30 seconds |
| Remove all measurement subscribers | Passed: waiting diagnostic, empty rejection reason, unavailable output, stable publication count; native raw demand zero and matcher/encoder inactive |
| Resume subscribers without restarting ROS | Passed: 54 coherent frame sets checked in 20 seconds |
| Optional rectified left/right images and CameraInfo, disparity | Passed: 78 coherent core sets in 30 seconds, plus detailed checks of eight retained optional sets |
| Stop and start native with ROS still running | Passed: PWM disabled on stop, runtime directory inode preserved, and the same ROS container recovered; 85 coherent sets across two native run identities |
| README depth-header and diagnostic commands | Passed using `/nuvo-entrypoint.sh` inside the existing Compose container |
| Calibration and control-file preservation | Passed: before/after SHA-256 checks matched |

Across the measurement, recovery, optional and restart phases, **300 coherent
core frame sets** passed the content checks. They covered encodings and dimensions,
shared timestamps/frame IDs, validity/NaN semantics, CameraInfo projection,
XYZ reprojection, capture identity and calibration authority. Optional checks
also verified image-grid projections and disparity-to-depth consistency.

Core receive rate was approximately 2.80–2.83 Hz; with all optional products it
was 2.63–2.73 Hz, using best-effort QoS. Median capture-to-observer latency was
465 ms for core output and 510 ms with optional products; observed maxima were
533 ms and 591 ms. This includes native matching, ROS conversion, DDS and observer
scheduling. It is not bridge overhead alone or a performance guarantee.

## Native startup repair

The installed native 0.1.0 daemon attempted a userspace GPIO request while the
kernel's PIO PWM driver already owned GPIO 17. The old installation lacked the
new preparation helper and unit integration. Upgrading the complete native
cohort, including its matching libcamera/IPA dependencies, fixed startup using
the ordinary native systemd service. No trigger implementation was added to ROS.
Native public beta 8 already includes PIO startup support; the newer development
build tested here also supplies measurement geometry and stable runtime mounts.

The Pi's existing explicit capture/preview policy was retained under the newer
capture-enabled gate. A legacy `--snapshot-dir` override was removed because it
requested native processing continuously without subscribers. Existing calibration
asset links were restored after package installation activated its default seed;
the original camera geometry and authority were verified before capture resumed.
Calibration hash remained
`3f096e19468426a733ef6c9632dd4642160aab9db00e8dd0e7ebc0f8b1d53e9a`, authority 0.
Unrelated robot/gateway service masks were preserved.

Follow [native startup](native-startup.md) before the [ROS quick start](../README.md#quick-start).
Published native installs use the native installer. For a manually selected
development cohort, back up configuration, calibration/identity state and asset
links first, preserve existing settings, and verify the calibration hash after
installation. A package's default seed is not a replacement for the unit's
existing geometry or validated calibration.

## Regression checks and evidence

The new DDS regression deliberately removes subscribers while a frame request
is in flight. The policy test also proves that pausing retains identity
watermarks and faults. The complete [AMD64 checks](https://github.com/arputhamips/nuvo-depth-ros/actions/runs/40526097711)
and [manual ARM64 checks](https://github.com/arputhamips/nuvo-depth-ros/actions/runs/40526169816)
passed, including HTTPS enabled/disabled container builds. Image publication was
disabled. The generated vendor SDK was unchanged and its manifest check passed.

Detailed local reports, test scripts, build logs and the transferred evidence
archive are retained under `output/audit/pi5-fixes-20261027/` (ignored by Git).
The archive SHA-256 is `3c1e688f8ff09daca4a29f5ec0001c30b01196ea5ea6b36f7a06d80cda46f7f2`.
The Pi retains its source checkout and reports under
`/home/cam/nuvo-ros-fixes-20261027/`. The fixed native service and ROS inspection
container remain running there; matching is idle until a measurement subscriber
connects. Provisional preview is explicitly enabled only in that test deployment.

This run covers same-Pi native IPC and DDS between containers. Ethernet/USB-C
acquisition, physical depth accuracy, cold-boot behavior and sustained operation
remain outside this test. No release or Git tag was created.
