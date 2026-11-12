#!/usr/bin/env python3
"""
config.py — CY2027 robot configuration.

Single source of truth for hardware wiring, drivetrain mode, and tuning
constants. Nothing else in this codebase should have a hardcoded pin,
motor ID, or serial setting — change it here as the chassis design firms up.
"""

from enum import Enum


class DriveMode(Enum):
    FOUR_WD = "four_wd"  # 4 independent drive motors, skid-steer (tank drive)
    STEERED = "steered"  # 1-2 drive motors + a dedicated steering servo (Ackermann-style)
    MASTER = "master"    # 1 motor mechanically drives all 4 wheels + a steering servo


# ── ACTIVE CONFIG — the one line to change once the chassis design settles ──
DRIVE_MODE = DriveMode.FOUR_WD

# ── SERIAL LINK to the motor/actuator microcontroller ────────────────────
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200

# ── DRIVE MOTOR IDS ───────────────────────────────────────────────────────
# FOUR_WD: one motor per wheel, independently driven.
DRIVE_MOTOR_IDS_4WD = {
    "front_left": 0,
    "front_right": 1,
    "rear_left": 2,
    "rear_right": 3,
}

# STEERED / MASTER: whichever motor id(s) actually spin the wheels.
#   STEERED — 1 id for RWD/FWD, or 2 if both axles are powered.
#   MASTER  — exactly 1 id: the single motor feeding the mechanical diff.
DRIVE_MOTOR_IDS_STEERED = [0]
STEER_SERVO_ID = 4
STEER_PULSE_MIN = 1000
STEER_PULSE_MID = 1500
STEER_PULSE_MAX = 2000

# ── CONTROLLER TUNING ──────────────────────────────────────────────────────
STICK_DEADZONE = 0.08  # fraction of full stick travel, 0-1
DRIVE_MAX_SPEED = 255  # top of the PWM range the firmware expects

# ── SUBSYSTEM SLOTS — not built yet, wire these up when they are ───────────
# ── SUBSYSTEM: FORKLIFT ──────────────────────────────────────────────────
# M<id> motor commands. Lift = raises carriage, tilt = angles the whole mast.
FORKLIFT_LIFT_ID = 5
FORKLIFT_TILT_ID = 6
FORKLIFT_LIFT_MAX_SPEED = 200   # PWM cap, 0-255
FORKLIFT_TILT_MAX_SPEED = 200
FORKLIFT_DEADZONE = 0.08        # fraction of stick travel

# ── SUBSYSTEM: CATAPULT ─────────────────────────────────────────────────
# A<id> servo commands. Winch is continuous-rotation, release is positional.
CATAPULT_WINCH_ID = 5
CATAPULT_RELEASE_ID = 6
CATAPULT_WINCH_STOP_PULSE = 1500    # µs — center = no motion
CATAPULT_WINCH_FULL_PULSE = 2000    # µs — full wind (flip to 1000 if reversed)
CATAPULT_RELEASE_REST_PULSE = 1500  # µs — pin engaged
CATAPULT_RELEASE_FIRE_PULSE = 1000  # µs — pin pulled
CATAPULT_RELEASE_HOLD_S = 0.25      # seconds to hold pin open