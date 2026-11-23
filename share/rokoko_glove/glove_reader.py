"""Reads live per-sensor fused orientation + acceleration from a Rokoko Smartglove II directly
over its USB-serial debug shell -- no Rokoko Studio, no official SDK/license.

How this works (found by live interactive exploration on 2026-11-23, not from any Rokoko
documentation -- Rokoko does not publish this interface and may change or remove it in a firmware
update without notice):

  The glove enumerates as idVendor=3921 idProduct=0631 "Rokoko Smartgloves II" with a CDC-ACM
  (virtual serial) interface. Opening it drops you into a Zephyr RTOS shell (`uart:~$`) running
  the glove's own firmware (seen: "Smartgloves FW: 3.4.0-release+0"). That shell has an `rbdp_shell`
  command group (RBDP = Rokoko's internal "Binary Data Protocol", the same system that feeds the
  official RGMP network stream) with a `subscribe processed_data` command that turns on a live log
  stream of every IMU's state. Enabling `print_imus` and each `enable_sensor <name>` (thumb, index,
  middle, ring, little, arm, hub) while disabling `print_internal_emf_data` /
  `print_external_emf_data` (electromagnetic hand-tracking debug data we don't need and which would
  otherwise flood the same stream) leaves clean lines of the form:

      <sensor> IMU data: acc: <ax> <ay> <az>, gyro (w,x,y,z): <qw> <qx> <qy> <qz>

  Despite the field being labeled "gyro (w,x,y,z)", the four values are a unit quaternion
  (w approx 1, x/y/z small when still) -- this is the glove's own on-device sensor-fusion output,
  not raw angular velocity. `acc` is linear acceleration in g (not yet confirmed against a
  known-1g static reference -- see CAVEATS in README.md). All 7 sensors cycle through at roughly
  80Hz combined (measured once, informally, not a guaranteed rate) on this specific glove/USB/host
  combination; the shell print path is a bottleneck (it reports "N messages dropped" between
  prints) relative to the internal sample rate, so this is NOT the glove's full native rate.

CAVEATS (do not build on this without re-reading README.md's CAVEATS section):
  - No absolute position data of any kind -- only orientation (+ a 3-axis acceleration reading)
    per sensor. Wrist/hand POSITION must come from elsewhere (e.g. this project's existing
    AprilTag/RealSense pipeline), not from this glove data alone.
  - Not validated against any ground truth yet (no comparison to a known rotation, no check that
    "acc" reads ~1g when stationary). Treat all numbers as unverified until that's done.
  - This is an undocumented debug interface, not a supported product feature. It may stop working
    after a firmware update, and depending on it long-term is a real risk, not a convenience.

Usage:
    python3 glove_reader.py                      # prints live values as you move the glove
    python3 glove_reader.py --port /dev/ttyACM0   # specific port instead of auto-detect
"""
from __future__ import annotations

import argparse
import re
import threading
import time
from dataclasses import dataclass, field

import serial
import serial.tools.list_ports

ROKOKO_VID = 0x3921
ROKOKO_PID = 0x0631
SENSOR_NAMES = ("thumb", "index", "middle", "ring", "little", "arm", "hub")

_LINE_RE = re.compile(
    r"(\w+) IMU data: acc: ([\d.eE+-]+) ([\d.eE+-]+) ([\d.eE+-]+), "
    r"gyro \(w,x,y,z\): ([\d.eE+-]+) ([\d.eE+-]+) ([\d.eE+-]+) ([\d.eE+-]+)"
)
_ANSI_RE = re.compile(rb"\x1b\[[0-9;]*[a-zA-Z]")


@dataclass
class SensorState:
    acc: tuple = (0.0, 0.0, 0.0)
    quat_wxyz: tuple = (1.0, 0.0, 0.0, 0.0)
    t_host: float = 0.0
    n_updates: int = 0


def find_port() -> str:
    for p in serial.tools.list_ports.comports():
        if p.vid == ROKOKO_VID and p.pid == ROKOKO_PID:
            return p.device
    raise RuntimeError(
        "No Rokoko Smartglove (idVendor=3921 idProduct=0631) found. Is it plugged in via USB? "
        "Check `lsusb` / `ls /dev/ttyACM*`.")


class GloveReader:
    """Opens the glove's debug shell, configures clean IMU-only streaming, and keeps `sensors`
    updated in a background thread. Does not assume anything about shell prompt timing beyond
    what was empirically observed -- each setup command is given a fixed settle time, not a
    prompt-matching state machine, because the shell's own reboot/banner timing was observed to
    vary (see README.md)."""

    def __init__(self, port: str | None = None, baud: int = 115200):
        self.port = port or find_port()
        self.ser = serial.Serial(self.port, baud, timeout=0.2)
        self.sensors = {name: SensorState() for name in SENSOR_NAMES}
        self._stop = threading.Event()
        self._thread = None
        self._buf = b""

    def _cmd(self, c: str, wait: float = 0.8) -> bytes:
        self.ser.reset_input_buffer()
        self.ser.write((c + "\r\n").encode())
        time.sleep(wait)
        return self.ser.read(8192)

    def setup(self):
        """Puts the glove into clean all-sensor IMU streaming mode. Idempotent -- safe to call
        on an already-configured glove (re-enabling an already-enabled sensor is a no-op).

        NOTE (see README.md CAVEATS): even once `list_display_options` confirms every sensor is
        enabled, the live `processed_data` stream was observed to reliably deliver only some
        sensors (thumb/index/middle/ring/hub) in testing -- "little" and "arm" rarely or never
        appeared in the stream despite `imus read_all` (a separate, one-shot, un-streamed command)
        reading all 7 fine. This is not fixed here; `hub` (hand/wrist orientation) is the one this
        project's solver actually needs, so it's the one this module is built around."""
        self._cmd("")  # wake/sync the line, discard whatever banner or stale prompt is pending
        for name in SENSOR_NAMES:
            self._cmd(f"rbdp_shell set_display_option enable_sensor {name}")
        self._cmd("rbdp_shell set_display_option print_imus 1")
        self._cmd("rbdp_shell set_display_option print_internal_emf_data 0")
        self._cmd("rbdp_shell set_display_option print_external_emf_data 0")
        self._cmd("rbdp_shell set_display_option print_every_nth 1")
        self._cmd("rbdp_shell set_display_option enable_printing 1")
        self._cmd("rbdp_shell subscribe processed_data", wait=0.8)

    def teardown(self):
        """Stops the stream and reboots the glove back to a clean, quiet state. Always call this
        when done -- leaving `processed_data` subscribed keeps the EMF amplifier/sampler running
        on the glove even after this process exits."""
        try:
            self._cmd("rbdp_shell set_display_option enable_printing 0")
            self._cmd("rbdp_shell unsubscribe processed_data")
            self._cmd("kernel reboot cold", wait=0.2)
        except Exception:
            pass

    def _parse_buffer(self):
        clean = _ANSI_RE.sub(b"", self._buf)
        if b"\n" not in clean:
            return  # no complete line yet -- wait for more data
        *complete_lines, leftover = clean.split(b"\n")
        self._buf = leftover
        for raw in complete_lines:
            text = raw.decode(errors="ignore")
            m = _LINE_RE.search(text)
            if not m:
                continue
            name = m.group(1)
            if name not in self.sensors:
                continue
            vals = [float(x) for x in m.groups()[1:]]
            st = self.sensors[name]
            st.acc = tuple(vals[0:3])
            st.quat_wxyz = tuple(vals[3:7])
            st.t_host = time.time()
            st.n_updates += 1

    def _run(self):
        while not self._stop.is_set():
            chunk = self.ser.read(4096)
            if chunk:
                self._buf += chunk
                self._parse_buffer()

    def start(self):
        self.setup()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        self.teardown()
        self.ser.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    args = ap.parse_args()

    reader = GloveReader(port=args.port)
    print(f"Connected on {reader.port}. Configuring stream...")
    reader.start()
    print("Streaming. Move the glove; press Ctrl+C to stop.\n")
    try:
        while True:
            time.sleep(0.5)
            line = []
            for name in SENSOR_NAMES:
                st = reader.sensors[name]
                age = time.time() - st.t_host if st.t_host else float("inf")
                q = st.quat_wxyz
                line.append(f"{name}: q=({q[0]:+.2f},{q[1]:+.2f},{q[2]:+.2f},{q[3]:+.2f}) "
                            f"age={age:.2f}s n={st.n_updates}")
            print(" | ".join(line))
    except KeyboardInterrupt:
        pass
    finally:
        print("\nStopping and rebooting glove to a clean state...")
        reader.stop()


if __name__ == "__main__":
    main()
