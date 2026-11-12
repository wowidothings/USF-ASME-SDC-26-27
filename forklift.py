#!/usr/bin/env python3
"""
forklift.py — forklift lift + tilt control.

Two speed-controlled motors sharing the RobotLink:

    lift:  raises/lowers the fork carriage
    tilt:  angles the whole lift assembly

Open-loop: the operator positions visually. To upgrade to closed-loop,
swap `link.drive(...)` for a position command — the interface to main.py
(update_lift / update_tilt / stop) stays the same.
"""

from __future__ import annotations

import config
from link import RobotLink


class Forklift:
    def __init__(self, link: RobotLink):
        self.link = link
        # Make sure we start stopped even if the firmware booted mid-command.
        self.link.drive(config.FORKLIFT_LIFT_ID, 0, force=True)
        self.link.drive(config.FORKLIFT_TILT_ID, 0, force=True)

    def update_lift(self, value: float) -> None:
        """value: -1.0 (full down) .. 1.0 (full up), 0 = stop."""
        value = self._shape(value)
        speed = int(value * config.FORKLIFT_LIFT_MAX_SPEED)
        self.link.drive(config.FORKLIFT_LIFT_ID, speed)

    def update_tilt(self, value: float) -> None:
        """value: -1.0 (one extreme) .. 1.0 (other), 0 = stop.
        Sign convention depends on which way the servo/motor is mounted —
        flip the sign here if the stick feels backwards."""
        value = self._shape(value)
        speed = int(value * config.FORKLIFT_TILT_MAX_SPEED)
        self.link.drive(config.FORKLIFT_TILT_ID, speed)

    def stop(self) -> None:
        self.link.drive(config.FORKLIFT_LIFT_ID, 0, force=True)
        self.link.drive(config.FORKLIFT_TILT_ID, 0, force=True)

    @staticmethod
    def _shape(value: float) -> float:
        """Deadzone + quadratic curve: fine control near center, full
        speed at the extremes. Makes precise positioning much easier
        without encoders."""
        if abs(value) < config.FORKLIFT_DEADZONE:
            return 0.0
        return value * abs(value)