# Release validation — 28 October 2026

ROS **v0.1.2** was built and published from
[`0ba19f2f3d0f96c0cf13bc0808fa24c2590a00e9`](https://github.com/arputhamips/nuvo-depth-ros/commit/0ba19f2f3d0f96c0cf13bc0808fa24c2590a00e9).
Its native counterpart is **v0.1.1-beta.9**, source
[`ce23f965f840d9a664f93bd71fa61aece3cf0101`](https://github.com/arputhamips/nuvo-depth/commit/ce23f965f840d9a664f93bd71fa61aece3cf0101).
Hosted checks passed; the new native-plus-ROS installation and argument matrix
**did not complete acceptance on the physical Pi**. Published source, tags and
images were retained unchanged.

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

## Physical Pi attempt

Inventory found the previous native beta 8 development cohort on a Pi 5 running
`6.18.33+rpt-rpi-2712`. Beta 9's signed platform manifest permits
`6.12.75+rpt-rpi-2712`; its platform guard was not bypassed. No new native or ROS
installation, boot edit, or new argument-matrix pass occurred.

The kernel reported repeated `mmc0` busy failures, `mmcblk0` read/write I/O
errors and an aborted EXT4 journal. The root filesystem reported `emergency_ro`,
and journald hit its watchdog. A Docker client was in uninterruptible sleep
inside `filemap_fault` / `folio_wait_bit_common`. A separate
`scheduling while atomic` warning during native startup included
`rp1_pio_sm_set_enabled` and `pwm_pio_rp1_apply`. These observations do not
identify whether the underlying cause is the card, power, kernel, or another
component, or establish a causal relationship between the two findings.

The inspected calibration remained provisional, authority `0`; it was neither
activated nor relabeled. Readable configuration, identity and calibration files
were copied for recovery; this is a partial copy, not a full filesystem backup.
Successful activation of a genuine validated per-unit bundle was unavailable
for acceptance in this attempt.

Private source logs and the prepared observer/matrix remain in the ignored
`output/audit/pi5-release-20261028/` directory. Host identifiers, credentials,
unit UUIDs and calibration hashes/content are omitted from this public record.

## Remaining acceptance

After storage recovery and booting a supported kernel:

1. Verify healthy filesystem/I/O, Docker responsiveness and native PIO startup;
   preserve the existing full/minimal profile and calibration before using the
   normal signed native installer and public ROS bootstrap.
2. Check fresh native/ROS capture identity and matching-grid geometry, numeric
   `nuvo-video` group access, unprivileged execution and the read-only runtime
   mount; verify preserved settings after installation, restart and reboot.
3. Exercise depth-only, points-only, left/right images, disparity and all outputs;
   requested 1/2/5/10 Hz ceilings; both launch files; YAML preservation and
   explicit overrides; helper dry-run/environment, namespace, domain and custom
   configuration; provisional gates and subscriber-driven idle/XYZ behavior.
4. Verify omitted calibration leaves native state unchanged and invalid explicit
   calibration fails without mutation. Run positive activation only with a
   genuine compatible accepted per-unit bundle and independent expected identity.

Requested cadence remains a delivery ceiling. This record makes no metric-depth,
electrical timing, sustained-throughput or production-qualification claim.
