# Start native capture before ROS

On the camera's Pi, the native `nuvo-depthd-native.service` owns the cameras,
calibration, matching and trigger. ROS needs only its Unix socket. Install native
Nuvo Depth through its [full](https://github.com/arputhamips/nuvo-depth/blob/main/docs/install.md)
or [minimal](https://github.com/arputhamips/nuvo-depth/blob/main/docs/minimal-install.md)
installation procedure before following the ROS quick start. Keep the installed
profile and preserve existing calibration, camera geometry and exposure settings.

## Check the native service

Run on the Pi host, from this ROS repository:

```bash
systemctl is-active nuvo-depthd-native.service
sudo journalctl -u nuvo-depthd-native.service -b -n 60 --no-pager
sudo python3 tools/check_native.py
```

The check is read-only. For the current Pi 5 PIO deployment it requires a running
native service, fresh status, `trigger_source: rp1-pio-pwm`, hardware timing and
the correct socket/group permissions. It prints the numeric `NUVO_VIDEO_GID`
needed by Compose. A provisional calibration notice is separate from startup
success: ROS still requires an explicit preview opt-in. Do not relabel or replace
calibration to make this check pass.

## GPIO 17 is already owned by PWM

`failed to request the trigger GPIO line` with GPIO 17 owned by `pwm_pio` means
an older daemon is trying to request a GPIO line already assigned to hardware
PWM. Giving the ROS container GPIO devices or privileged mode cannot fix this.
Do not unload the working PWM overlay or switch to a userspace trigger.

Use a coherent native package build that includes all of these:

- A daemon that discovers RP1 PIO PWM by device-tree compatibility and GPIO.
- `/usr/lib/nuvo-depth/nuvo-trigger-prepare` and its systemd service.
- A native unit that runs preparation before capture, grants access to the
  prepared channel and disables it after stopping.

The ROS release number does not identify the installed native version. Check
native packages with `dpkg-query -W 'nuvo-depth*'`. Native **v0.1.1-beta.8 already
includes PIO startup support**; a host still running native 0.1.0 has not been
upgraded just because the ROS v0.1.0 image was installed. Native source `98fcb4c29bcefd5398f6d2ddc669224917a40f31`
contains this startup support and preserves the runtime directory across service
restarts. The public native beta 8 and a newer development build may have the same
package version, so verify source provenance as well as the version string.

Use the native project's approved installer for published packages. Development
packages come from a manually selected **ARM64 Debian packages** Actions run;
they are test artifacts, not a new public release. Keep the complete package
cohort for your current full/minimal profile. Do not copy one `depthd` executable
out of another installation or replace the unit with a robot-specific launch.

## Masked or stopped services

Inspect deliberate masks and local overrides before restarting:

```bash
systemctl show nuvo-depthd-native.service -p UnitFileState -p DropInPaths
ls -l /run/systemd/system/nuvo-depthd-native.service /run/systemd/system/nuvo-trigger-prepare.service
sudo systemctl cat nuvo-depthd-native.service
```

If you previously masked these two services for maintenance, have installed the
compatible native packages, and now intend to start capture:

```bash
sudo systemctl unmask --runtime nuvo-depthd-native.service nuvo-trigger-prepare.service
sudo systemctl daemon-reload
sudo systemctl start nuvo-depthd-native.service
sudo python3 tools/check_native.py
```

A persistent `/etc/systemd/system` mask must be resolved through the native
setup that created it. Leave unrelated robot/gateway service policy alone.
For a fresh or intentionally capture-disabled installation, use native setup to
enable capture; do not remove its `ConditionPathExists` checks. For failure logs,
also inspect `journalctl -u nuvo-trigger-prepare.service -b -n 40 --no-pager`.

Local `ExecStart=` overrides can retain obsolete command-line flags or bypass
new package behavior. Compare them with the installed unit, preserve calibration
paths and preview policy, and remove only overrides superseded by the package.
In particular, a legacy `--snapshot-dir` argument can request native processing
continuously even when ROS has no subscribers. It is not needed by this adapter;
remove it only when that legacy snapshot consumer is no longer intended.

After native startup passes, continue with the [ROS quick start](../README.md#quick-start).
When upgrading from a legacy unit that replaces `/run/nuvo-depth`, recreate the
ROS container after the native socket is ready. The current native unit preserves
that directory across ordinary stop/start cycles.
