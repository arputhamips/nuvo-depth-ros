# Development validation — 26 October 2026

This record covers the measurement-validation, acquisition-policy, optional
product and deployment changes following the [25 October record](validation-20261025.md).
Evidence comes from Linux amd64 host/container tests under WSL2 and a native
ARM64 hosted CI runner. It does not qualify Raspberry Pi hardware, sustained
camera performance or metric accuracy.
The runtime candidate is
`3afe3cdffa04d9d2b5a41492d90ac0b43295e526`; later documentation updates do not
change its tested production sources. The canonical native source checkpoint is
`b653f06d35c4fcac7b966a3f02622eb0d58e4ec6`.

## Confirmed SDK evidence

SDK snapshot at this checkpoint:
`54ffce563db1bf50637e97a6bd5c2f00a090b16cb4009dbeb31c646c3ae913be`.
The [source manifest](../nuvo_depth_client_vendor/upstream/SOURCE-MANIFEST.json)
records the base commit and exported file hashes; the content hash identifies
the exact tested exported files independently of the base commit.

| Check | Result |
|---|---|
| Canonical SDK, HTTPS enabled | 4/4 CTests passed |
| Canonical SDK, HTTPS disabled | 4/4 CTests passed |
| Standalone exported SDK, HTTPS disabled | 4/4 CTests passed |
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
accuracy. Final container checks are recorded below.

The native systemd fixtures use the pinned ROS base image and UID 10001. Both
preserved the runtime-directory inode, removed generated socket/status/snapshot
artifacts and preserved an unrelated file. Reports are in the core repository's
`output/native-runtime-lifecycle/*.json` and
`/root/nuvo-runtime-lifecycle-results`. This supplements the older same-directory
socket-replacement fixture rather than reinterpreting it as a systemd lifecycle
test. Older installed units and manual directory replacement still need the
[container recreation procedure](reference.md#local-only-operation).

## Final amd64 candidate checks

| Check | Result |
|---|---|
| ROS conversion, identity-pin, continuity and clock-policy tests | 34 tests, zero failures/errors/skips |
| DDS pin recovery, 0.5-second no-frame expiry, reconnect/lazy continuity and fault-latch behavior | Passed |
| Optional-product demand masks and validity-masked ROS disparity | Passed |
| Depth-only/image-only replay and missing image geometry with usable depth | Passed; per-product availability and conversion reason reported |
| Docker HTTPS ON/OFF builds | Each built and passed 34 tests |
| Docker HTTPS dependency/ELF boundary checks | Passed for both images |
| Read-only runtime mount, numeric group access, UID 10001 and reconnect | Passed for both images |
| Local memfd acquisition with Docker network disabled and private IPC | Passed for both images |
| Full-size depth, mono/RGB rectified images, CameraInfo, disparity and XYZ through each final container | Passed with associated capture stamps and the shipped Cyclone profile |
| Subscriber demand release | Both images changed optional mask 47 → depth-only 40 → no demand; no encoded-preview requests |
| Clean shutdown | HTTPS ON 0.422 seconds; OFF 0.396 seconds; both exit 0 |

The colcon count includes 31 GoogleTest cases and three CTest wrapper results.

Final local image identities:

| Build | Image ID |
|---|---|
| HTTPS ON | `sha256:70b564c45119b0dc889af34d01adb04283eabd6cc2d5f75fb1322d4dcc3da992` |
| HTTPS OFF | `sha256:23167818b4c3ba9ab57f1eede42750d117ab8982ba419dfeae8001634b7e1505` |

Evidence is retained in the ROS checkout's ignored
`output/audit/universal-ros/contract-results.json` and
`output/audit/products-deployment/{docker-on,docker-off,host-fullsize}/`.
The adjacent source snapshot records production-source hashes. One additional
negative clock-step assertion in an existing test was added after the image
snapshot; the final source workspace passed all 34 tests, and the production
sources match the images.

The HTTPS-OFF build explicitly requests no HTTPS APT packages in either stage
and has no direct SDK/component ELF links to SSL, crypto, Boost or c-ares. The
pinned ROS base still contains SSL through its own dependencies. This is a
local/replay-only client build, not a claim that the entire image lacks TLS.

Both container full-size tests used 728 x 544 grids, a 1584128-byte depth image
and a 4752384-byte XYZ cloud. Host `rmem_max`/`wmem_max` remained 212992 and
`ipfrag_high_thresh` remained 4194304; IP reassembly counters did not grow.
The network-disabled check establishes the separate Unix-socket acquisition
path; DDS delivery to other machines still requires networking.

## Confirmed hosted ARM64 evidence

The manually dispatched [native ARM64 workflow](https://github.com/arputhamips/nuvo-depth-ros/actions/runs/40383105604)
passed all three jobs on `ubuntu-24.04-arm` at runtime candidate
`3afe3cdffa04d9d2b5a41492d90ac0b43295e526`. The Jazzy source job passed SDK
snapshot/export checks, SDK CTests, ROS tests, DDS integration and full-size DDS
delivery. Both Docker variants reported architecture `arm64` and 34 tests with
zero errors, failures or skips.

Both ARM64 images also passed the unprivileged host-socket/reconnect contract,
network-disabled/private-IPC acquisition, full-size container delivery and
dependency/ELF audit. Runtime UID was 10001 with numeric group access. Full-size
728 x 544 products included the 1584128-byte depth image and 4752384-byte cloud;
optional demand changed 47 → 40 → no demand, with both release checks passing.
These are hosted synthetic-source checks, separate from Pi camera operation.

| Build | Hosted image ID | Deployment evidence and runtime bundle |
|---|---|---|
| HTTPS ON | `sha256:7e4b00d328e0811dc151d1f6b7500da95f7985f22a6a711d5fec995e7df6c023` | [ON artifact](https://github.com/arputhamips/nuvo-depth-ros/actions/runs/40383105604/artifacts/12680016203) |
| HTTPS OFF | `sha256:d99f9555ac001672dd439b88fb943b7eb56d39316444fd850d126f0c977339ae` | [OFF artifact](https://github.com/arputhamips/nuvo-depth-ros/actions/runs/40383105604/artifacts/12679287983) |

The [Jazzy contract artifact](https://github.com/arputhamips/nuvo-depth-ros/actions/runs/40383105604/artifacts/12679247750)
retains source-test/full-size reports and the SDK manifest. Runtime-bundle
checksums were independently verified against each artifact's `SHA256SUMS`:

| Bundle | SHA-256 |
|---|---|
| `nuvo-depth-ros-jazzy-arm64-https-ON.tar.gz` | `af86ec211c07fd81b46f36c150099e527db3e248380cd87e4bdc0ae902236128` |
| `nuvo-depth-ros-jazzy-arm64-https-OFF.tar.gz` | `ec0538b4b8a8b102fc872864951df76003ebf3ac37de6fd0c699d1a35ddb9aa9` |

The local verification summary is retained in
`output/audit/hosted-arm64-40383105604/verification-summary.json`.

## Remaining qualification

| Check | Status |
|---|---|
| Real Pi/camera endurance, resource use, latency and physical accuracy | Not yet established |

Receipt of small metadata/CameraInfo messages alone does not demonstrate
image/cloud transport. A separate local Fast DDS host experiment delivered no
DDS messages. Fast DDS is not qualified by this record and no unqualified
example is shipped. The node remains RMW-agnostic.

The manual ARM64 workflow produces validation artifacts. A hosted ARM64 result
does not establish operation on the Pi or camera. Routine changes do not create
tags or releases.

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
