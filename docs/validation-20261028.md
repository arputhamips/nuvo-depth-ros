# Release validation — 28 October 2026

ROS **v0.1.2** was built and published from
[`0ba19f2f3d0f96c0cf13bc0808fa24c2590a00e9`](https://github.com/arputhamips/nuvo-depth-ros/commit/0ba19f2f3d0f96c0cf13bc0808fa24c2590a00e9).
Its native counterpart is **v0.1.1-beta.9**, source
[`ce23f965f840d9a664f93bd71fa61aece3cf0101`](https://github.com/arputhamips/nuvo-depth/commit/ce23f965f840d9a664f93bd71fa61aece3cf0101).
Hosted checks passed. After recovery and selection of the supported kernel,
both public installers completed on the physical Pi. Live argument testing
exposed a host DDS receive-buffer limit; the v0.1.3 host-setup patch addresses
that finding. All 23 v0.1.2 argument-matrix phases passed after host setup;
the final patch installation is still being checked. Published v0.1.2 source,
tag and image remain unchanged.

## Completed release checks

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
unit with the expected ordering. Reboot persistence and the final published
bootstrap remain part of final patch acceptance.

The v0.1.2 matrix passed all 23 planned phases: individual and combined outputs,
1/2/5/10 Hz ceilings, YAML preservation and explicit overrides, inspection
launch, idle behavior, helper dry-run/environment/deployment, and negative
calibration-file cases through direct launch and helper deployment. Invalid
archives and the actual provisional single-camera YAML were rejected without
changing native configuration or calibration. The four rejection cases are
negative tests, not successful bundle activation.

Private source logs and the prepared observer/matrix remain in the ignored
`output/audit/pi5-release-20261028/` directory. Host identifiers, credentials,
unit UUIDs and calibration hashes/content are omitted from this public record.

## Remaining acceptance

Before final patch acceptance:

1. Install the final ROS patch through its public bootstrap and verify the
   automatic host receive-limit setup, including reboot persistence.
2. Recheck native/ROS identity, geometry and preserved settings after reboot.
3. Exercise depth-only, points-only, left/right images, disparity and all outputs;
   requested 1/2/5/10 Hz ceilings; both launch files; YAML preservation and
   explicit overrides; helper dry-run/environment, namespace, domain and custom
   configuration; provisional gates and subscriber-driven idle/XYZ behavior.
4. Verify omitted calibration leaves native state unchanged and invalid explicit
   calibration fails without mutation. Run positive activation only with a
   genuine compatible accepted per-unit bundle and independent expected identity.

Requested cadence remains a delivery ceiling. This record makes no metric-depth,
electrical timing, sustained-throughput or production-qualification claim.
