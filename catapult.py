#!/usr/bin/env python3
"""
catapult.py — catapult wind + release control.

    winch:   continuous-rotation servo. 1500 µs = stop, 2000 µs = full
             wind. The operator holds the trigger to wind and releases
             to stop, judging compression visually. Because it's a
             continuous servo there is no position feedback — wind
             amount is whatever the operator decided.

    release: positional servo. Rest holds the pin; fire pulls it. The
             release is a one-shot: pull, hold briefly, return to rest.
             Auto-return means the operator can't accidentally leave the
             pin open.

If the winch turns out to need more torque than a hobby servo can give,
replace the A<id> calls with link.drive(config.CATAPULT_WINCH_ID, speed)
and set CATAPULT_WINCH_ID to a motor id in config.py. Everything else
in this file stays the same.
"""

from __future__ import annotations

import asyncio

import config
from link import RobotLink


class Catapult:
    def __init__(self, link: RobotLink):
        self.link = link
        # Start with winch stopped and pin engaged.
        self.link.actuator(config.CATAPULT_WINCH_ID,
                           config.CATAPULT_WINCH_STOP_PULSE, force=True)
        self.link.actuator(config.CATAPULT_RELEASE_ID,
                           config.CATAPULT_RELEASE_REST_PULSE, force=True)
        self._release_task: asyncio.Task | None = None

    # ── wind ──────────────────────────────────────────────────────────
    def wind(self, amount: float) -> None:
        """amount: 0.0 (stop) .. 1.0 (full wind speed)."""
        if amount < 0.05:
            self.stop_wind()
            return
        pulse = int(config.CATAPULT_WINCH_STOP_PULSE +
                    amount * (config.CATAPULT_WINCH_FULL_PULSE -
                              config.CATAPULT_WINCH_STOP_PULSE))
        self.link.actuator(config.CATAPULT_WINCH_ID, pulse)

    def stop_wind(self) -> None:
        self.link.actuator(config.CATAPULT_WINCH_ID,
                           config.CATAPULT_WINCH_STOP_PULSE, force=True)

    # ── release ───────────────────────────────────────────────────────
    def release(self) -> None:
        """Fire the catapult. Idempotent: pressing the button again
        while a release is in flight does nothing."""
        if self._release_task and not self._release_task.done():
            return
        self._release_task = asyncio.create_task(self._release_sequence())

    async def _release_sequence(self) -> None:
        # Stop the winch first so we never fire with the winder running.
        self.stop_wind()
        self.link.actuator(config.CATAPULT_RELEASE_ID,
                           config.CATAPULT_RELEASE_FIRE_PULSE, force=True)
        await asyncio.sleep(config.CATAPULT_RELEASE_HOLD_S)
        self.link.actuator(config.CATAPULT_RELEASE_ID,
                           config.CATAPULT_RELEASE_REST_PULSE, force=True)

    # ── shutdown ──────────────────────────────────────────────────────
    def stop(self) -> None:
        self.stop_wind()
        if self._release_task and not self._release_task.done():
            self._release_task.cancel()
        # Software backstop for stored energy — a mechanical lock or
        # normally-closed solenoid is still the right answer.
        self.link.actuator(config.CATAPULT_RELEASE_ID,
                           config.CATAPULT_RELEASE_REST_PULSE, force=True)