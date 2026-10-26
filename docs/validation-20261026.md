# Development validation — 26 October 2026

This record covers the measurement-validation, acquisition-policy, optional
product and deployment changes following the [25 October record](validation-20261025.md).
Evidence comes from Linux amd64 host/container tests under WSL2. It does not
qualify Raspberry Pi hardware, sustained camera performance or metric accuracy.

## Confirmed SDK evidence

SDK snapshot at this checkpoint:
`54ffce563db1bf50637e97a6bd5c2f00a090b16cb4009dbeb31c646c3ae913be`.
The [source manifest](../nuvo_depth_client_vendor/upstream/SOURCE-MANIFEST.json)
records the base commit and exported file hashes; the content hash identifies
the tested snapshot, including changes beyond that base commit.

| Check | Result |
|---|---|
| Canonical SDK, HTTPS enabled | 4/4 CTests passed |
| Canonical SDK, HTTPS disabled | 4/4 CTests passed |
| Standalone exported SDK | 4/4 CTests passed |
| Real beta 6 NVR1 recording and four native wire fixtures | Decode/measurement compatibility passed |
| Native serializer → SDK/native recording interoperability | Passed |

The SDK checks typed depth/disparity ranges, quantization and valid ROI;
accepted samples, validity/NaN conventions and producer configuration errors;
and calibrated depth/disparity consistency where geometry and both products
are available. Legacy recordings without declarations remain supported without
inventing missing limits. Depth-only demand still selects depth and validity.
Recorded-camera compatibility does not establish scene accuracy.

Local evidence in the `NuvoDepthSession2404` WSL environment:

- `/root/nuvo-sdk-measurements-on/Testing/Temporary/LastTest.log`
- `/root/nuvo-sdk-measurements-off/Testing/Temporary/LastTest.log`
- `/root/nuvo-sdk-measurements-export-build/Testing/Temporary/LastTest.log`
- `/root/nuvo-sdk-measurements-native/`

## Confirmed host deployment evidence

| Check | Result |
|---|---|
| Full-size 728 x 544 depth, rectified images, disparity and XYZ over host DDS | Passed with the MTU-safe Cyclone profile |
| Full/minimal native runtime-directory lifecycle fixtures | Both passed six run identities, including restart, stop/start, crash and failed startup recovery |
| Native runtime-artifact cleanup with trigger-stop failure | Passed; cleanup completed while systemd retained the failure result |

The first full-size failure came from oversized Cyclone UDP datagrams
(14720 bytes) crossing a 1500-byte interface MTU. The investigation observed
`ReasmFails=15177`, `ReasmOKs=0`. It was not a requirement to permanently raise
host socket buffers. With automatic interface selection,
`MaxMessageSize=1400B` and `FragmentSize=1200B`, all products arrived while
`rmem_max` and `wmem_max` were restored to their original 212992 values.
See the [shipped transport profile](reference.md#dds-transport).

The host full-size report is
`/root/nuvo-fullsize-results/fullsize-host.json`, with its adjacent log. It records
optional-product demand mask 47, depth-only mask 40 and release of optional/all
demand, including a 4752384-byte XYZ cloud and 1584128-byte depth image.
The tested Cyclone DDS version was 0.10.5. These synthetic fixtures establish
host transport and conversion behavior, not sustained Pi camera throughput or
accuracy. Final container checks remain separate below.

The native systemd fixtures use the pinned ROS base image and UID 10001. Both
preserved the runtime-directory inode, removed generated socket/status/snapshot
artifacts and preserved an unrelated file. Reports are in the core repository's
`output/native-runtime-lifecycle/*.json` and
`/root/nuvo-runtime-lifecycle-results`. This supplements the older same-directory
socket-replacement fixture rather than reinterpreting it as a systemd lifecycle
test. Older installed units and manual directory replacement still need the
[container recreation procedure](reference.md#local-only-operation).

## Final candidate checks still pending

The implementation is still under final review. Earlier development passes do
not qualify later corrections. Record the final candidate's exact source/image
identity and results before treating these rows as passed:

| Check | Status at this checkpoint |
|---|---|
| ROS conversion, identity-pin, continuity and clock-policy tests | Final rerun pending |
| DDS pin recovery, freshness, reconnect/lazy continuity and fault-latch behavior | Final rerun pending |
| Optional-product demand masks and validity-masked ROS disparity | Final rerun pending |
| Docker HTTPS ON/OFF builds and dependency/ELF boundary checks | Final candidate pending |
| Full-size depth, images, disparity and XYZ through the final container | Final candidate pending; host profile test passed above |
| Native ARM64 Docker/runtime qualification | Not yet established |
| Real Pi/camera endurance, resource use, latency and physical accuracy | Not yet established |

Receipt of small metadata/CameraInfo messages alone does not demonstrate
image/cloud transport. A separate local Fast DDS host experiment delivered no
DDS messages. Fast DDS is not qualified by this record and no unqualified
example is shipped. The node remains RMW-agnostic.

The manual ARM64 workflow provides a qualification path and artifacts; its
presence is not evidence that it has run or passed. Routine changes do not
create tags or releases.

## Deployment and commissioning limits

Identity/calibration pins, restart policy and clock policy are startup settings.
Provisional preview retains unqualified native authority, and inspection topics
and diagnostics use a separate namespace. Driver freshness/qualification is a
measurement contract; downstream consumers retain their own deadlines and robot
health decisions. See the [integration guide](integration.md).

There is no released minimum native version covering this complete change set.
Record the approved native revision and selected product capabilities alongside
the driver commit, SDK hash, image identity, resolved package versions and
calibration hash/authority. Preserve existing validated per-unit bundles.
