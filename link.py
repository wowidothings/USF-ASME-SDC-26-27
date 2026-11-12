#!/usr/bin/env python3
"""
link.py — serial link to the drive/actuator microcontroller.

New protocol for CY2027. Last year's firmware only knew one drive motor
plus 8 servo channels ("S<id>:<pw>", "D<speed>:<dir>", "M<id>" for the dump
motors) — it doesn't speak this and needs a rewrite to match:

    M<id>:<signed_speed>\n   drive motor `id`, speed -255..255, sign = direction, 0 = stop
    A<id>:<pulse_us>\n       actuator/servo `id`, pulse width in microseconds (1000-2000)

Every drive motor and every servo/actuator gets its own id. That's what
lets 4WD, the forklift, the catapult, and eventually a climber all share
one link without stepping on each other.
"""

from __future__ import annotations

import serial

import config


class RobotLink:
    def __init__(self, port: str = config.SERIAL_PORT, baud: int = config.SERIAL_BAUD):
        self._ser = serial.Serial(port, baud, timeout=0.01)
        self._last_drive: dict[int, int] = {}
        self._last_actuator: dict[int, int] = {}

    def drive(self, motor_id: int, speed: int, force: bool = False) -> None:
        """speed: -255..255, sign = direction, 0 = stop."""
        speed = max(-255, min(255, int(speed)))
        last = self._last_drive.get(motor_id)
        if not force and last is not None and abs(last - speed) < 2:
            return
        self._last_drive[motor_id] = speed
        self._ser.write(f"M{motor_id}:{speed}\n".encode())

    def actuator(self, actuator_id: int, pulse_us: int, force: bool = False) -> None:
        """pulse_us: microseconds, 1000-2000."""
        pulse_us = max(1000, min(2000, int(pulse_us)))
        last = self._last_actuator.get(actuator_id)
        if not force and last is not None and abs(last - pulse_us) < 5:
            return
        self._last_actuator[actuator_id] = pulse_us
        self._ser.write(f"A{actuator_id}:{pulse_us}\n".encode())

    def stop_all(self, motor_ids) -> None:
        for mid in motor_ids:
            self.drive(mid, 0, force=True)

    def close(self) -> None:
        self._ser.close()
