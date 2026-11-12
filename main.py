#!/usr/bin/env python3
"""
main.py — CY2027 robot entry point.

Wired up so far: Bluetooth controller -> drivetrain (4WD / steered / master,
per config.py) -> serial link to the motor/actuator microcontroller.

Forklift, catapult, and climber are NOT wired up yet. config.py has open
slots for their actuator IDs, and controller_input.py exposes every raw
input as a named slot (buttons, triggers, d-pad) that the drivetrain isn't
using. Adding a subsystem later: pick its actuator ID(s) in config.py, write
its control logic in its own file, then register it below with

    self.controller.on(SLOT_NAME, handler)

No changes needed to controller_input.py itself.

Dependencies: pip install pyserial, sudo apt install python3-evdev
"""

from __future__ import annotations

import asyncio

import config
import controller_input as ci
from link import RobotLink
from drivetrain import build_drivetrain
from forklift import Forklift
from catapult import Catapult


def apply_deadzone(value: float, deadzone: float) -> float:
    return 0.0 if abs(value) < deadzone else value


class Robot:
    def __init__(self):
        self.link = RobotLink()
        self.drivetrain = build_drivetrain(self.link)
        self.forklift = Forklift(self.link)
        self.catapult = Catapult(self.link)
        self.controller = ci.ControllerInput()

        self._throttle = 0.0
        self._turn = 0.0

        self._bind_drive_controls()
        self._bind_forklift_controls()
        self._bind_catapult_controls()

        # ── OPEN SLOTS ──────────────────────────────────────────────────
        # Forklift, catapult, climber go here once they exist, e.g.:
        #   self.controller.on(ci.DPAD_Y, self._on_forklift_dpad)
        #   self.controller.on(ci.BTN_A, self._on_catapult_release)
        # Their actuator IDs live in config.py (currently None).

    def _bind_drive_controls(self) -> None:
        # Left stick: throttle (Y) + turn (X). Move these to the right stick
        # or a different axis pair here if that ends up feeling better.
        self.controller.on(ci.LEFT_STICK_Y, self._on_throttle)
        self.controller.on(ci.LEFT_STICK_X, self._on_turn)

    def _on_throttle(self, value: float) -> None:
        # Most pads report stick-forward as negative -- flip it here.
        self._throttle = -apply_deadzone(value, config.STICK_DEADZONE)
        self.drivetrain.update(self._throttle, self._turn)

    def _on_turn(self, value: float) -> None:
        self._turn = apply_deadzone(value, config.STICK_DEADZONE)
        self.drivetrain.update(self._throttle, self._turn)

        # ── forklift ──────────────────────────────────────────────────────
    def _bind_forklift_controls(self) -> None:
        # Right stick: Y = lift, X = tilt. The forklift class applies its
        # own deadzone and speed curve, so we pass raw stick values.
        self.controller.on(ci.RIGHT_STICK_Y, self._on_forklift_lift)
        self.controller.on(ci.RIGHT_STICK_X, self._on_forklift_tilt)

    def _on_forklift_lift(self, value: float) -> None:
        # Pads report stick-forward as negative — flip so up = raise.
        self.forklift.update_lift(-value)

    def _on_forklift_tilt(self, value: float) -> None:
        self.forklift.update_tilt(value)

    # ── catapult ──────────────────────────────────────────────────────
    def _bind_catapult_controls(self) -> None:
        # Right trigger winds, BTN_A releases.
        self.controller.on(ci.RIGHT_TRIGGER, self._on_catapult_wind)
        self.controller.on(ci.BTN_A, self._on_catapult_release)

    def _on_catapult_wind(self, value: float) -> None:
        self.catapult.wind(value)

    def _on_catapult_release(self, value: bool) -> None:
        if value:               # fire on press, ignore the release edge
            self.catapult.release()

    async def run(self) -> None:
        await self.controller.run()

    def shutdown(self) -> None:
        self.drivetrain.stop()
        self.forklift.stop()
        self.catapult.stop()
        self.link.close()


def main() -> None:
    robot = Robot()
    print(f"[ok] Drive mode: {config.DRIVE_MODE.value}")
    print("[ok] Waiting for controller input... (Ctrl-C to stop)")
    try:
        asyncio.run(robot.run())
    except KeyboardInterrupt:
        print("\n[stop] Ctrl-C -- shutting down.")
    finally:
        robot.shutdown()
        print("[ok] Shut down cleanly.")


if __name__ == "__main__":
    main()
