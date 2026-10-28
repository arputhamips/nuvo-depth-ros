# Boundaries and release discipline

- Canonical acquisition, decoding and NVR1 support live in Nuvo Depth/sdk/cpp.
  Refresh the vendor snapshot with its exporter; do not hand-edit it.
- Native Nuvo Depth owns cameras, matching and calibration. Default connections
  must not change device configuration or require recalibration. An explicitly
  supplied calibration_file may request native bundle validation/activation via
  its local control API; never implement calibration activation in the adapter.
- Preserve calibration hash/authority, matching-grid geometry and capture identity.
- Default deployment: native daemon on Pi OS Lite 64-bit; Ubuntu Jazzy container
  on the same Pi 5, with its runtime directory and numeric nuvo-video GID.
- XYZ generation belongs here, is optional, and runs only with subscribers.
- Routine CI produces artifacts. ARM64 package builds are manual. No tags,
  releases or publication without an explicit user request/approval.
- Do not advertise Pi hardware qualification based on host/container tests.
