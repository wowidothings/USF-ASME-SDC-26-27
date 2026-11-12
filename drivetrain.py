#!/usr/bin/env python3
"""
drivetrain.py — motor control for getting the robot around.

Three interchangeable drivetrains, all driven by the same two numbers:
    throttle: -1.0 (full reverse) .. 1.0 (full forward)
    turn:     -1.0 (full left)    .. 1.0 (full right)

config.DRIVE_MODE picks which one main.py builds. Changing chassis designs
later means changing one line in config.py, not rewriting drive logic.
"""

from __future__ import annotations

import config
from link import RobotLink


class SkidSteerDrivetrain:
    """4 independent drive motors, tank/skid-steer mixing. config.DriveMode.FOUR_WD."""

    def __init__(self, link: RobotLink, motor_ids: dict[str, int]):
        self.link = link
        self.ids = motor_ids

    def update(self, throttle: float, turn: float) -> None:
        left = max(-1.0, min(1.0, throttle + turn))
        right = max(-1.0, min(1.0, throttle - turn))
        left_speed = int(left * config.DRIVE_MAX_SPEED)
        right_speed = int(right * config.DRIVE_MAX_SPEED)
        self.link.drive(self.ids["front_left"], left_speed)
        self.link.drive(self.ids["rear_left"], left_speed)
        self.link.drive(self.ids["front_right"], right_speed)
        self.link.drive(self.ids["rear_right"], right_speed)

    def stop(self) -> None:
        self.link.stop_all(self.ids.values())


class SteeredDrivetrain:
    """
    1-2 drive motors + a steering servo (config.DriveMode.STEERED, Ackermann-style),
    or a single motor mechanically driving all 4 wheels (config.DriveMode.MASTER).
    Both are the same control problem — one throttle, one steering angle — so
    they share this class; only the motor ID list in config.py differs.
    """

    def __init__(self, link: RobotLink, drive_motor_ids: list[int], steer_servo_id: int):
        self.link = link
        self.drive_ids = drive_motor_ids
        self.steer_id = steer_servo_id

    def update(self, throttle: float, turn: float) -> None:
        speed = int(max(-1.0, min(1.0, throttle)) * config.DRIVE_MAX_SPEED)
        for mid in self.drive_ids:
            self.link.drive(mid, speed)

        turn = max(-1.0, min(1.0, turn))
        mid_pw = config.STEER_PULSE_MID
        span = (config.STEER_PULSE_MAX - mid_pw) if turn >= 0 else (mid_pw - config.STEER_PULSE_MIN)
        self.link.actuator(self.steer_id, int(mid_pw + turn * span))

    def stop(self) -> None:
        self.link.stop_all(self.drive_ids)
        self.link.actuator(self.steer_id, config.STEER_PULSE_MID, force=True)


def build_drivetrain(link: RobotLink):
    if config.DRIVE_MODE is config.DriveMode.FOUR_WD:
        return SkidSteerDrivetrain(link, config.DRIVE_MOTOR_IDS_4WD)
    return SteeredDrivetrain(link, config.DRIVE_MOTOR_IDS_STEERED, config.STEER_SERVO_ID)
