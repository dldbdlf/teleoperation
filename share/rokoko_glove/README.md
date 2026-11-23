# Rokoko Smartglove — direct USB capture, no Studio/SDK (2026-11-23)

The team's Rokoko Studio account is on the free Starter plan, which does not include Custom
Streaming / livestreaming (confirmed against rokoko.com/pricing and Rokoko's own plan-comparison
support article: that's a Plus-tier feature, ~$20-28/mo). The official SDK (sdk.rokoko.com, RCSP +
RGMP protocols) is a separate, license-gated path requiring a request through
`rokoko.com/use-cases/robotics` with unknown cost/terms. Both were ruled out for this session in
favor of talking to the glove's own firmware directly.

## What the glove actually is, electrically

Plugging the Smartglove II into USB enumerates it as `idVendor=3921 idProduct=0631`, a CDC-ACM
(virtual serial) device — on this machine it shows up as `/dev/ttyACM0` or `/dev/ttyACM1`
(the number increments on every reboot of the glove since udev doesn't reuse the old node until a
cleanup cycle; always re-detect, never hardcode the number — `glove_reader.find_port()` does this
by VID/PID).

Opening that serial port does **not** stream data — it drops you into an interactive **Zephyr RTOS
shell** (`uart:~$`), the same debug console the firmware team presumably uses internally. Observed
banner: `Smartglove FW: 3.4.0-release+0 (git sha=c11ed9e684f2) - build @ 2026-09-18T07:42:49+0000`.
This is **not a documented or supported interface** — Rokoko does not publish it, and a firmware
update could change or remove it without notice. Treat everything below as reverse-engineered by
interactive exploration on this one unit, on this one firmware version, not as a guaranteed API.

`help` at the prompt lists every shell command group. The relevant ones:
- `imus` — `list` (names the 7 onboard IMUs), `read_all` (one-shot raw accel+gyro, all 7, always
  reliable), `status`.
- `rbdp_shell` — controls RBDP ("Rokoko Binary/Data Protocol", the internal system that also feeds
  the official RGMP network stream). `subscribe processed_data` turns on a live log stream;
  `set_display_option` controls what that stream prints (`enable_sensor <name>`, `print_imus`,
  `print_internal_emf_data` / `print_external_emf_data`, `print_every_nth`, `enable_printing`).
- `magnetometer`, `bmi270` — lower-level sensor-chip commands, not used here.
- `kernel reboot cold` — the clean way to reset the glove back to a quiet state after experimenting
  (always do this when done — see CAVEATS).

## Two different data sources, not interchangeable

| | `imus read_all` (poll) | `rbdp_shell subscribe processed_data` (stream) |
|---|---|---|
| Coverage | **All 7 sensors, every call** | thumb/index/middle/hub reliable; **ring/little/arm rarely or never appear** (reproduced across multiple sessions/reboots — not a fluke) |
| Data | Raw accel (g) + raw gyro (rad/s), **no orientation** | Already-fused **quaternion** per sensor (mislabeled `gyro (w,x,y,z)` in the log line — it's w,x,y,z of a unit quaternion, not angular velocity) + an accel field in different, unidentified units (~1000x the poll path's values — not yet reconciled) |
| Measured rate | **3.2 Hz** (30 polls / 9.46s) — shell round-trip-bound, not sensor-rate-bound | **~8-12 Hz for `hub`** (measured: 199 updates / 25s and 124 updates / ~14s in separate runs) |

3.2 Hz is too slow for teleoperation. The stream is fast enough for `hub`, which is also the only
sensor this project's solver actually needs (NERO is a 7-DOF arm with no fingers to drive — finger
orientation from thumb/index/middle is a bonus for later, not a requirement). `glove_reader.py` and
`glove_ros_node.py` are built around the stream path for exactly this reason, and don't try to fix
`little`/`arm` or reconcile the stream's accel units -- not needed for the current goal.

## Files

- **`glove_reader.py`** — `GloveReader` class: finds the port by VID/PID, runs the shell setup
  sequence, parses `<sensor> IMU data: acc: ... gyro (w,x,y,z): ...` lines from the live stream in
  a background thread, exposes `.sensors[name].quat_wxyz` / `.acc` / `.t_host` / `.n_updates`. Also
  runnable standalone (`python3 glove_reader.py`) to print live values while you move the glove —
  use this first to sanity-check the hardware before touching ROS.
- **`glove_ros_node.py`** — wraps `GloveReader` in an `rclpy` node, publishes
  `geometry_msgs/QuaternionStamped` on `/rokoko_glove/<sensor>/orientation` for each of the 7
  sensors (only once that sensor has actually produced a sample — a topic with zero publishes for
  `arm`/`little`/`ring` in a given session is expected, not a bug). Logs a per-sensor OK/STALE
  status line every 5s.

Both require the `dialout` group for serial port access (`sudo usermod -aG dialout $USER`, then
`newgrp dialout` or a fresh login) and `pip3 install --user pyserial`.

## CAVEATS — read before building on this

1. **Undocumented interface.** This is the glove's own debug shell, not an API Rokoko supports or
   documents. It may disappear in a firmware update. Don't depend on it for anything
   safety-critical, and don't be surprised if a future firmware makes this README wrong.
2. **No position data at all.** Every sensor here gives orientation (+ an accel reading); none
   gives 3D position. Driving the NERO solver needs a wrist *position* (`Frame.b_W` in
   `share/sim/baselines.py`) as well as orientation — that has to come from somewhere else (this
   project's existing AprilTag/RealSense pipeline in `share/apriltag_gt/` is the known-working
   source) or from a forward-kinematics approximation (fixed shoulder anchor + measured upper-
   arm/forearm lengths + the `arm`/`hub` orientations) that **has not been built or validated yet**.
3. **Not validated against any ground truth.** No check yet that the quaternion tracks a known
   rotation correctly, or that `imus read_all`'s accel reads ~1g when stationary (it does look
   plausible -- e.g. thumb ~0.98g net -- but "looks plausible" is not the same as measured). Do
   this before trusting the orientation numbers for anything quantitative, the same way the
   apriltag_gt work insisted on an independent accuracy check rather than stopping at repeatability.
4. **`little`/`arm` not streaming is unexplained**, not just unfixed. `imus read_all` proves the
   hardware itself works for those two; something in the RBDP publisher/subscription path filters
   or drops them specifically. Worth revisiting if finger or forearm data becomes a real
   requirement later.
5. **Always `kernel reboot cold` when done.** `rbdp_shell subscribe processed_data` leaves the
   glove's EMF amplifier/sampler running even after your script exits; `GloveReader.stop()` /
   `GloveRosNode.destroy_node()` both do this automatically, but if a script crashes without
   calling it, reboot the glove by hand before the next session (`screen /dev/ttyACM0 115200` or
   the one-liner in glove_reader.py's `teardown()`).
6. **Setup takes ~10s** (13 shell round-trips at ~0.8s settle time each). A caller with its own
   timeout (an external `timeout` command, a test harness) needs to budget for that — a run killed
   mid-setup partially configures the glove and leaves it in whatever state the last-sent command
   produced.

## Not done yet

- The Frame-building bridge into `share/sim/baselines.py`'s solver (needs the position-source
  decision in CAVEATS #2 above — worth deciding deliberately rather than guessing).
- Reconciling or just living with the stream's unidentified accel units.
- Any ground-truth validation (CAVEATS #3).
- Investigating why `little`/`arm` don't stream (CAVEATS #4), if they turn out to matter later.
