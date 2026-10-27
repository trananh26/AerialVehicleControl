"""Standalone fake-Arduino Pozyx sender used to validate the SITL beacon path.

This script plays the role of the ``IndoorLoiter.ino`` sketch without needing
any Gazebo simulation: it publishes a fixed beacon layout and a slowly moving
vehicle position over the Pozyx binary protocol, so that ArduPilot SITL's
``AP_Beacon_Pozyx`` backend and the EKF3 range-beacon fusion can be exercised
in isolation.

Usage::

    python3 fake_pozyx_sender.py --port 5761

Then start SITL with::

    arducopter --model x500 --serial1=tcp:5761:wait --defaults beacon.parm
"""

from __future__ import annotations

import argparse
import math
import os
import socket
import struct
import sys
import time

# Allow running straight from a source checkout without installing the package.
sys.path.insert(
    0, os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
)

from uwb_pozyx_bridge.pozyx_protocol import (
    encode_beacon_config,
    encode_beacon_distance,
    encode_vehicle_position,
)


# Anchor layout matching Tools/Pozyx/IndoorLoiter/IndoorLoiter.ino:
#   anchors_x = {0, 10000, 0, 10000}
#   anchors_y = {0, 0, 10000, 10000}
#   heights   = {-1200, -1200, -1200, -1200}
#
# The values below are expressed **relative to the beacon origin**, which is
# placed at the centre of the 10 m x 10 m anchor square (the position where
# the vehicle spawns).  This mirrors the Gazebo setup where the four anchors
# sit at the corners of a 10 m x 10 m area and the drone starts in the
# middle.
#
# Note on the anchor height: the Arduino sketch uses -1200 mm, meaning the
# anchors are 1.2 m above the vehicle's starting altitude.  The Pozyx backend
# converts this to NED as z = +1.2 (down is positive), so the anchors sit
# 1.2 m above the beacon origin.
DEFAULT_ANCHORS = (
    (0, -5000.0, -5000.0, -1200.0),
    (1, 5000.0, -5000.0, -1200.0),
    (2, 5000.0, 5000.0, -1200.0),
    (3, -5000.0, 5000.0, -1200.0),
)


def distance_to_anchor(
    anchor: tuple[float, float, float], pos: tuple[float, float, float]
) -> float:
    """3D range between an anchor and the vehicle, in millimetres."""
    dx = anchor[0] - pos[0]
    dy = anchor[1] - pos[1]
    dz = anchor[2] - pos[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


class FakePozyxSender:
    """Emulates the Arduino sketch's serial output."""

    def __init__(
        self,
        host: str,
        port: int,
        anchors=DEFAULT_ANCHORS,
        position_error_mm: int = 50,
        rate_hz: float = 20.0,
        orbit_radius_m: float = 2.0,
        orbit_period_s: float = 20.0,
        static: bool = False,
        static_pos_m: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        self.host = host
        self.port = port
        self.anchors = anchors
        self.position_error_mm = position_error_mm
        self.period = 1.0 / rate_hz
        self.orbit_radius_m = orbit_radius_m
        self.orbit_period_s = orbit_period_s
        self.static = static
        self.static_x = static_pos_m[0] * 1000.0
        self.static_y = static_pos_m[1] * 1000.0
        self.static_z = static_pos_m[2] * 1000.0
        self.sock: socket.socket | None = None
        self._sent_configs = False

    # -- connection ----------------------------------------------------

    def connect(self, timeout: float = 30.0) -> None:
        """Connect to the SITL serial port, retrying until it appears."""
        deadline = time.time() + timeout
        while True:
            try:
                sock = socket.create_connection((self.host, self.port), timeout=5.0)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.sock = sock
                print(
                    f"[fake_pozyx] connected to {self.host}:{self.port}",
                    flush=True,
                )
                return
            except OSError as exc:
                if time.time() > deadline:
                    raise RuntimeError(
                        f"could not connect to {self.host}:{self.port} within "
                        f"{timeout:.0f}s: {exc}"
                    ) from exc
                print(
                    f"[fake_pozyx] waiting for {self.host}:{self.port} ({exc})",
                    flush=True,
                )
                time.sleep(1.0)

    def _send(self, data: bytes) -> None:
        assert self.sock is not None
        self.sock.sendall(data)

    # -- payload generation --------------------------------------------

    def vehicle_position(self, t: float) -> tuple[float, float, float]:
        """Orbit the vehicle around the centre of the anchor square.

        The beacon origin is the centre of the 10 m x 10 m anchor layout, so
        the vehicle position is expressed relative to it, in ENU millimetres.
        """
        if self.static:
            return (self.static_x, self.static_y, self.static_z)
        phase = 2.0 * math.pi * t / self.orbit_period_s
        x_mm = self.orbit_radius_m * 1000.0 * math.cos(phase)
        y_mm = self.orbit_radius_m * 1000.0 * math.sin(phase)
        z_mm = 0.0
        return x_mm, y_mm, z_mm

    def build_frames(self, t: float) -> bytes:
        """Build the frames for one update cycle."""
        frames = bytearray()

        # The Arduino sketch sends the anchor configuration during start-up.
        if not self._sent_configs:
            for beacon_id, x_mm, y_mm, z_mm in self.anchors:
                frames += encode_beacon_config(
                    beacon_id, len(self.anchors), int(x_mm), int(y_mm), int(z_mm)
                )
            self._sent_configs = True

        pos = self.vehicle_position(t)

        # Distances to each anchor.
        for beacon_id, x_mm, y_mm, z_mm in self.anchors:
            dist_mm = distance_to_anchor((x_mm, y_mm, z_mm), pos)
            frames += encode_beacon_distance(beacon_id, int(round(dist_mm)))

        # Vehicle position fix.
        frames += encode_vehicle_position(
            int(round(pos[0])),
            int(round(pos[1])),
            int(round(pos[2])),
            self.position_error_mm,
        )

        return bytes(frames)

    # -- main loop -----------------------------------------------------

    def run(self, duration_s: float | None = None) -> None:
        self.connect()
        start = time.time()
        next_tick = start

        try:
            while duration_s is None or (time.time() - start) < duration_s:
                now = time.time()
                frames = self.build_frames(now - start)
                self._send(frames)

                next_tick += self.period
                sleep_for = next_tick - time.time()
                if sleep_for > 0:
                    time.sleep(sleep_for)
                else:
                    next_tick = time.time()
        except (BrokenPipeError, ConnectionResetError) as exc:
            raise RuntimeError(f"connection to SITL lost: {exc}") from exc
        finally:
            if self.sock is not None:
                self.sock.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Fake Pozyx Arduino for SITL beacon validation"
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5761)
    parser.add_argument("--rate", type=float, default=20.0, help="send rate in Hz")
    parser.add_argument(
        "--position-error",
        type=int,
        default=50,
        help="positioning error estimate in millimetres (must stay < 1000)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="stop after this many seconds (default: run forever)",
    )
    parser.add_argument(
        "--orbit-radius",
        type=float,
        default=2.0,
        help="radius of the simulated vehicle orbit in metres",
    )
    parser.add_argument(
        "--orbit-period",
        type=float,
        default=20.0,
        help="period of the simulated vehicle orbit in seconds",
    )
    parser.add_argument(
        "--static",
        action="store_true",
        help="hold the vehicle still instead of orbiting",
    )
    parser.add_argument(
        "--static-pos",
        type=float,
        nargs=3,
        default=[0.0, 0.0, 0.0],
        metavar=("X", "Y", "Z"),
        help="static vehicle position in metres, relative to the beacon origin",
    )
    args = parser.parse_args(argv)

    sender = FakePozyxSender(
        host=args.host,
        port=args.port,
        position_error_mm=args.position_error,
        rate_hz=args.rate,
        orbit_radius_m=args.orbit_radius,
        orbit_period_s=args.orbit_period,
        static=args.static,
        static_pos_m=tuple(args.static_pos),
    )

    try:
        sender.run(duration_s=args.duration)
    except RuntimeError as exc:
        print(f"[fake_pozyx] error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
