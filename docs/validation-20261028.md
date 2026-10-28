# Release validation — 28 October 2026

ROS **v0.1.2** introduced the startup controls and was published from
[`0ba19f2f3d0f96c0cf13bc0808fa24c2590a00e9`](https://github.com/arputhamips/nuvo-depth-ros/commit/0ba19f2f3d0f96c0cf13bc0808fa24c2590a00e9).
Its native counterpart is **v0.1.1-beta.9**, source
[`ce23f965f840d9a664f93bd71fa61aece3cf0101`](https://github.com/arputhamips/nuvo-depth/commit/ce23f965f840d9a664f93bd71fa61aece3cf0101).
After recovery and selection of the supported kernel, both public installers
completed on the physical Pi. Live testing exposed a host DDS receive-buffer
limit. ROS **v0.1.3**, source
[`eafd85a0ec9e631fc6971c0ba147c71397af1c8e`](https://github.com/arputhamips/nuvo-depth-ros/commit/eafd85a0ec9e631fc6971c0ba147c71397af1c8e),
adds automatic host setup and a larger DDS receive request. Its public
installation, eventual reboot recovery, **25 argument cases** and observation
of the installed service passed. The initial startup limitation and subsequent
storage investigation are recorded below.
Published v0.1.2 source, tag and image remain unchanged.

## Completed release checks

- [ROS v0.1.3 workflow 40762727583](https://github.com/arputhamips/nuvo-depth-ros/actions/runs/40762727583)
  passed all seven AMD64/ARM64 source, HTTPS ON/OFF deployment and publication
  jobs. Anonymous full pulls of both architectures, labels, package versions,
  launch arguments and installed DDS profile passed. All 91 archived source
  files match the tested Git tree. The fixed image manifest is
  `sha256:8d627c926ee2018216237bde0b4b19ee2f1bbd9632d340a7c04204e6bbb0f2fb`.
- [ROS release workflow 40748681358](https://github.com/arputhamips/nuvo-depth-ros/actions/runs/40748681358)
  passed all seven jobs: Jazzy source/component/DDS checks on AMD64 and ARM64,
  Docker deployment with HTTPS enabled and disabled on both architectures, and
  image publication. Anonymous pulls of both images, package versions and an
  AMD64 launch-argument smoke check also passed.
- [Native signed ARM64 workflow 40747535974](https://github.com/arputhamips/nuvo-depth/actions/runs/40747535974)
  passed the full and minimal builds, with 69/69 and 66/66 CTest entries
  respectively. Signatures, package metadata, asset hashes and archived source
  matched the release records.

These results establish host/container and artifact contracts. The
[27 October Pi report](validation-pi5-20261027.md) is historical evidence for an
earlier software cohort and does not establish v0.1.2 hardware acceptance.

## Physical Pi recovery and installation

Inventory found the previous native beta 8 development cohort on a Pi 5 running
`6.18.33+rpt-rpi-2712`. Beta 9's signed platform manifest permits
`6.12.75+rpt-rpi-2712`; its platform guard was not bypassed.

The kernel reported repeated `mmc0` busy failures, `mmcblk0` read/write I/O
errors and an aborted EXT4 journal. The root filesystem reported `emergency_ro`,
and journald hit its watchdog. A Docker client was in uninterruptible sleep
inside `filemap_fault` / `folio_wait_bit_common`. A separate
`scheduling while atomic` warning during native startup included
`rp1_pio_sm_set_enabled` and `pwm_pio_rp1_apply`. These observations do not
identify whether the underlying cause is the card, power, kernel, or another
component, or establish a causal relationship between the two findings.

After the operator checked storage/power and restarted, Docker and storage
became responsive. The already-installed supported kernel and its staged boot
assets were verified, the normal boot configuration was backed up, and the
approved kernel selection was applied. Subsequent normal and installer reboots
returned to `6.12.75+rpt-rpi-2712`. No fresh MMC errors or PIO scheduling warning
were observed during these checks. This does not establish the cause of the
earlier failure or long-term storage reliability.

The inspected calibration remained provisional, authority `0`; it was neither
activated nor relabeled. Readable configuration, identity and calibration files
were copied for recovery; this initial copy was partial. Once storage was
responsive, a root-only backup of the upgrade inputs was also saved on the Pi.
The existing provisional YAML pair was preserved byte-for-byte in a stable
root outside the package-managed `current` tree, and the existing native service
override was pointed there. Existing camera overlays were adopted into the
installer's managed block without changing their hardware values. See the
[native legacy-upgrade preflight](https://github.com/arputhamips/nuvo-depth/blob/main/docs/installer-reference.md)
for this older hand-configured installation case.

The public beta 9 **full-profile** installer completed through its normal
interactive flow and reboot; the existing profile was retained. Its receipt
reported `complete`, native capture and hardware PIO triggering started, and
`nuvoctl ros environment --json` reported ready. The matching-grid geometry and
local calibration-control socket were available. The public ROS v0.1.2 bootstrap
then pulled the released image and started its unprivileged container with the
discovered numeric `nuvo-video` group and read-only native runtime mount. All
four installed ROS packages reported `0.1.2`; image digest matched the release.
Device identity, provisional calibration bytes/hash/authority and existing
camera settings were preserved. No source build or manual `.env` edit was
needed on the Pi.

Successful activation of a genuine validated per-unit bundle remains untested:
only the existing provisional calibration is available. A single-camera YAML
must not be wrapped or relabeled as an accepted native stereo bundle.

## DDS receive-buffer finding

The first depth-only observation received too few coherent same-stamp product
sets. Instrumented repetitions used the same released image, observer QoS,
cache size and acceptance criteria:

| Host receive maximum | DDS receive request | Observation | Coherent sets / 20 s | Host UDP receive-buffer errors |
| --- | --- | --- | --- | --- |
| 212,992 bytes (original) | Default 1 MiB | Depth | 27 | +17,118 |
| 8,388,608 bytes | Default 1 MiB | Depth | 56 | +427 |
| 8,388,608 bytes | Default 1 MiB | All outputs | 50 | +822 |
| 8,388,608 bytes | Explicit 8 MiB | All outputs | 52 | 0 |

Only `net.core.rmem_max` changed for the first three observations; DDS endpoints
were recreated. Default socket buffers, send limits and Cyclone DDS XML stayed
the same. A repeated all-output run still received only 40 coherent sets from
53 metadata/image-info samples. The final comparison additionally requested
8 MiB through `Internal/SocketReceiveBufferSize` in both endpoints' XML;
the minimum remained `default` to permit startup on other hosts. It received
52 matching samples on every product topic with no added receive-buffer errors.
The counters are host-wide and do not identify individual ROS packets; the
paired delivery observations support the receive-buffer diagnosis. These are
bounded observations, not a guarantee of lossless best-effort transport.
Depth frame checks, projection, identity and all-output association passed.
All-output delivery was approximately 2.6 Hz with the existing native matching
settings and a requested 10 Hz ceiling. This is not a 10 Hz throughput result.

The v0.1.3 helper prepares an 8 MiB receive maximum automatically, preserving
higher administrator limits. Its root-owned boot unit runs after normal sysctl
setup and before Docker. Each explicit helper launch recreates ROS sockets,
including retries after an interrupted installation. Camera configuration is
outside this host setup.

The candidate helper was also exercised on this Pi with receive maxima of
212,992, 8,388,608 and 16,777,216 bytes: it raised the first and preserved the
other two. Default receive/send limits were unchanged. Dry-run created no unit
and changed no limit; real setup produced an enabled, active root-owned `0644`
unit with the expected ordering.

The v0.1.2 matrix passed all 23 planned phases: individual and combined outputs,
1/2/5/10 Hz ceilings, YAML preservation and explicit overrides, inspection
launch, idle behavior, helper dry-run/environment/deployment, and negative
calibration-file cases through direct launch and helper deployment. Invalid
archives and the actual provisional single-camera YAML were rejected without
changing native configuration or calibration. The four rejection cases are
negative tests, not successful bundle activation.

## Published v0.1.3 installation and reboot

The Pi downloaded the bootstrap from its public GitHub `v0.1.3` tag; its
SHA-256 matched the tagged source and prepared release asset. To exercise
automatic setup, the audit-created host unit was removed and the original
212,992-byte receive maximum restored. Public bootstrap dry-run left the
container, user YAML and low limit unchanged and created no host unit.
The actual bootstrap then pulled the released image digest, discovered the
native numeric group, installed the host unit, raised the maximum to 8 MiB
and recreated the unprivileged ROS container. All four package versions were
`0.1.3`. Existing native files, device identity, calibration and ROS YAML were
preserved; namespace, domain, all output selections and requested cadence were
verified in the deployment.

After a normal reboot, the Pi returned on the supported kernel with the same
image, native calibration and configuration. The 8 MiB maximum persisted.
The host setup unit completed in **78 ms**, entering active state at boot time
71.776 s, before Docker started at 179.324 s.

**Startup was slow and required four native/trigger-preparation retries.**
The final trigger preparation completed at 217.967 s and native capture became
active at 220.070 s. Systemd recorded 3 min 43.349 s total startup; the network
return was observed about six minutes after the reboot request. Wi-Fi initially
failed association before retrying, `NetworkManager-wait-online` failed after
72.194 s, and several unrelated services also had long startup delays. Docker
took 43.864 s to start. Previous shutdown logs show normal service stops,
filesystem synchronization and journal closure. No fresh kernel MMC I/O error
or `scheduling while atomic` warning appeared in the inspected boot journal.

The original logs did not establish the cause of the broad startup stalls or
identify a concrete native trigger defect. Service/source review found no
ordering cycle involving the new DDS unit. No network profile or startup
timeout was changed to conceal these retries. This is evidence of eventual
automatic recovery, not repeatable fast startup or cold-start qualification.

### Subsequent startup investigation

A later instrumented reboot reproduced three native retries and a 188.584 s
systemd boot. A temporary in-memory sampler continued at approximately 0.5 s
intervals while `depthd` waited in `filemap_fault`, PID 1 waited on ext4/journal
writes, and MMC workers waited for I/O completion. Three 29–33 s windows had
zero completed SD reads or writes and 94–98% CPU I/O wait. The trigger helper
was waiting for its configuration-dump child to load libraries; it had not yet
reached PIO setup.

The SD controller's `err_stats` recorded **20 data timeouts**, despite no
matching MMC I/O error in the boot journal. This establishes storage-path
latency behind the observed startup failures. It does not distinguish a card,
controller, power or signalling fault. No network timeout, camera configuration,
calibration or service deadline was changed to hide the failures.

Native source hardening adds a configuration-only executable using the same
parser, immediate trigger-stage diagnostics and optional read-only MMC error
reporting. These source changes are separate from immutable native beta 9 and
ROS v0.1.3 releases. A device-specific SD compatibility experiment is not a
general deployment default or proof that the underlying storage is healthy.

The clock-only experiment did not lower the controller's actual 200 MHz clock.
SD-slot-only 3.3 V high-speed mode then produced an actual 50 MHz clock while
Wi-Fi remained on its separate 1.8 V DDR50 interface. Two normal reboots passed:

| Native / SD setup | Systemd boot | Native restarts | Recorded MMC errors |
| --- | ---: | ---: | ---: |
| Published beta 9 with SD high-speed workaround | 53.333 s | 0 | 0 |
| Native source fixes with the same workaround, temporary tracing removed | 52.639 s | 0 | 0 |

The latter used native source
`9b4be87882e82d0a529f827db7d566299a2997bd`. Its new utility was compiled on the Pi;
configuration JSON, digests and successful check output matched the existing
daemon byte-for-byte. The matching helper, native unit and diagnostics module
were installed as a test candidate. The configuration query completed in
0.395 s, trigger preparation finished at 12.807 s, and native became active at
29.421 s. The released ROS container recovered automatically with zero restarts.

Before and after the final reboot, the installed ROS service passed all-output
observation: **51 coherent frame sets in each 20-second run**, approximately
2.62–2.64 Hz at the unchanged 10 Hz ceiling. Checks covered depth, validity,
CameraInfo, XYZ, left/right images, disparity, timestamps and preserved capture/
calibration identity. Both runs added zero UDP receive-buffer errors. Native
returned to idle after unsubscribe; trigger/frame-completion counters remained
zero. Configuration, calibration and installed candidate checksums passed.

The optional overlay source exactly matched the deployed overlay after
compilation. Temporary strace and sampler units were removed. No network profile,
camera calibration or release asset was changed. The
[native investigation](https://github.com/arputhamips/nuvo-depth/blob/main/docs/startup-storage-validation-20261028.md)
contains the storage evidence, optional workaround and full/minimal ARM64 build
results. It distinguishes this tested device configuration from general hardware
qualification.

## Original v0.1.3 argument matrix, before the startup fixes

All **25 cases passed** on the published ARM64 image after the original release
reboot: 21 ordinary
cases and four expected calibration-input rejections. The suite covered every
output individually and combined; 1/2/5/10 Hz ceilings; YAML preservation and
explicit overrides; namespace/domain/custom configuration; both launch files,
including the default inspection YAML; helper dry-run and deployment; lazy
idle; default provisional gating and an explicit `false` override of YAML
`true`. Invalid archive and actual provisional single-camera YAML inputs were
rejected through both direct launch and helper paths without native mutation.

| Observation | Coherent frame sets in 20 s | Approximate received rate |
| --- | ---: | ---: |
| Depth only | 58 | 2.95 Hz |
| Points only | 58 | 2.94 Hz |
| Left / right only | 200 / 199 | 10 Hz |
| All outputs | 52 on every product topic | 2.65 Hz |
| Depth with 1 Hz / 2 Hz ceiling | 20 / 39 | 1 Hz / 2 Hz |
| Installed service, all outputs | 51 | 2.62 Hz |

Checks covered standard topic types and publisher selection, matching-grid
dimensions/projection, depth/validity/NaN representation, XYZ and disparity
consistency, frame IDs, timestamps and capture/calibration identity. They do
not establish physical distance accuracy. Host-wide UDP counters recorded
**zero additional receive-buffer errors** over the full matrix; this is a
bounded observation, not a lossless-delivery guarantee.

The installed service remained running with zero container restarts after that
original matrix. Native trigger/frame-completion error counters were zero, and native
demand and matching returned to idle after subscribers left. Native's restart
count remained four on that earlier boot, matching its initial retries; it did
not restart during the matrix. The later startup-fix reboot above had zero
restarts and was checked with two all-output runs, rather than repeating all
25 argument cases. Audit containers were removed. Calibration/configuration
fingerprints remained unchanged. The container remains configured for all
outputs, an explicit provisional-preview opt-in and a 10 Hz delivery ceiling.

Sanitized release assets provide the [case-by-case matrix](https://github.com/arputhamips/nuvo-depth-ros/releases/download/v0.1.3/pi5-argument-matrix.md),
[machine-readable matrix](https://github.com/arputhamips/nuvo-depth-ros/releases/download/v0.1.3/pi5-argument-matrix.json)
and [installation/reboot record](https://github.com/arputhamips/nuvo-depth-ros/releases/download/v0.1.3/pi5-install-verification.json).

Private source logs and the prepared observer/matrix remain in the ignored
`output/audit/pi5-release-20261028/` directory. Host identifiers, credentials,
unit UUIDs and calibration hashes/content are omitted from this public record.

## Qualification limits

- Successful activation of a genuine accepted stereo calibration bundle was
  **not run**: only the preserved provisional pair was available. Negative
  rejection tests do not substitute for positive activation.
- The physical test upgraded an existing full-profile installation. Minimal
  package/installer contracts passed hosted checks; this was not a fresh minimal
  hardware installation. The matrix samples output combinations and rate values,
  not every possible parameter value or deployment topology.
- The native startup retries were subsequently traced to SD/MMC I/O stalls;
  two software reboots passed with the device-specific SD workaround. The
  underlying physical fault and long-term storage health remain unqualified.
  Power-cycle behavior, long-duration operation, independent
  Ethernet/USB-C hardware combinations and alternate DDS implementations remain
  outside this physical test.

Requested cadence remains a delivery ceiling. This record makes no metric-depth,
electrical timing, sustained-throughput or production-qualification claim.
