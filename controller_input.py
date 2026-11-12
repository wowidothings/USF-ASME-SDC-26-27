#!/usr/bin/env python3
"""
controller_input.py — Bluetooth game controller input (Stadia/PS4/Xbox-style).

This module only knows about raw controller inputs — it has no idea a
drivetrain, forklift, or catapult exists. Anything downstream hooks in with

    controller.on(SLOT_NAME, callback)

That's the "open slots": adding a subsystem later means registering a new
callback somewhere else, not editing this file.

Axis values are normalized using each device's own reported min/max (evdev
AbsInfo), not hardcoded per-controller-model constants — last year's scripts
hardcoded 0-255 in one place and -32768..32767 in another and disagreed with
each other about which pad was actually plugged in.

    Sticks   → -1.0 .. 1.0, 0 at rest
    Triggers →  0.0 .. 1.0, 0 at rest
    D-pad    → -1, 0, 1 (already discrete)
    Buttons  → True (pressed) / False (released)
"""

from __future__ import annotations

import evdev #ignore error till pi
from evdev import InputDevice, ecodes

# ── SLOT NAMES — register handlers against these strings ──────────────────
LEFT_STICK_X = "left_stick_x"
LEFT_STICK_Y = "left_stick_y"
RIGHT_STICK_X = "right_stick_x"
RIGHT_STICK_Y = "right_stick_y"
LEFT_TRIGGER = "left_trigger"
RIGHT_TRIGGER = "right_trigger"
DPAD_X = "dpad_x"
DPAD_Y = "dpad_y"
BTN_A = "btn_a"
BTN_B = "btn_b"
BTN_X = "btn_x"
BTN_Y = "btn_y"
BTN_L1 = "btn_l1"
BTN_R1 = "btn_r1"
BTN_L3 = "btn_l3"
BTN_R3 = "btn_r3"

_STICK_CODES = {
    ecodes.ABS_X: LEFT_STICK_X,
    ecodes.ABS_Y: LEFT_STICK_Y,
    ecodes.ABS_RX: RIGHT_STICK_X,
    ecodes.ABS_RY: RIGHT_STICK_Y,
}
_TRIGGER_CODES = {
    ecodes.ABS_Z: LEFT_TRIGGER,
    ecodes.ABS_RZ: RIGHT_TRIGGER,
}
_HAT_CODES = {
    ecodes.ABS_HAT0X: DPAD_X,
    ecodes.ABS_HAT0Y: DPAD_Y,
}
_BTN_CODES = {
    ecodes.BTN_SOUTH: BTN_A,
    ecodes.BTN_EAST: BTN_B,
    ecodes.BTN_NORTH: BTN_Y,
    ecodes.BTN_WEST: BTN_X,
    ecodes.BTN_TL: BTN_L1,
    ecodes.BTN_TR: BTN_R1,
    ecodes.BTN_THUMBL: BTN_L3,
    ecodes.BTN_THUMBR: BTN_R3,
}

_NAME_KEYWORDS = ("wireless controller", "stadia", "xbox", "x-box",
                  "microsoft", "dualshock", "dualsense")
_NAME_EXCLUDE = ("touchpad", "touch pad", "mouse", "motion")


def find_controller() -> InputDevice:
    devices = [InputDevice(p) for p in evdev.list_devices()]
    for d in devices:
        name = d.name.lower()
        if any(k in name for k in _NAME_KEYWORDS) and not any(e in name for e in _NAME_EXCLUDE):
            return d
    for d in devices:
        name = d.name.lower()
        if ecodes.EV_ABS in d.capabilities() and not any(e in name for e in _NAME_EXCLUDE):
            print(f"[warn] No known controller found by name, using: {d.name}")
            return d
    raise RuntimeError(
        "No controller found. Pair it first:\n"
        "  bluetoothctl → power on → scan on → pair <MAC> → connect <MAC>"
    )


class ControllerInput:
    def __init__(self, device: InputDevice | None = None):
        self.device = device or find_controller()
        self._handlers: dict[str, list] = {}
        self._abs_ranges = self._read_abs_ranges()

    def _read_abs_ranges(self) -> dict[int, tuple[int, int]]:
        ranges = {}
        for code, absinfo in self.device.capabilities(absinfo=True).get(ecodes.EV_ABS, []):
            ranges[code] = (absinfo.min, absinfo.max)
        return ranges

    def on(self, slot: str, callback) -> None:
        """Register `callback(value)` to fire whenever `slot`'s input changes."""
        self._handlers.setdefault(slot, []).append(callback)

    def _emit(self, slot: str, value) -> None:
        for cb in self._handlers.get(slot, ()):
            cb(value)

    def _normalize_stick(self, code: int, raw: int) -> float:
        lo, hi = self._abs_ranges.get(code, (-32768, 32767))
        center = (lo + hi) / 2
        half = (hi - lo) / 2 or 1
        return max(-1.0, min(1.0, (raw - center) / half))

    def _normalize_trigger(self, code: int, raw: int) -> float:
        lo, hi = self._abs_ranges.get(code, (0, 255))
        span = (hi - lo) or 1
        return max(0.0, min(1.0, (raw - lo) / span))

    async def run(self) -> None:
        print(f"[ok] Controller: {self.device.name}")
        async for event in self.device.async_read_loop():
            if event.type == ecodes.EV_ABS:
                code, val = event.code, event.value
                if code in _STICK_CODES:
                    self._emit(_STICK_CODES[code], self._normalize_stick(code, val))
                elif code in _TRIGGER_CODES:
                    self._emit(_TRIGGER_CODES[code], self._normalize_trigger(code, val))
                elif code in _HAT_CODES:
                    self._emit(_HAT_CODES[code], val)
            elif event.type == ecodes.EV_KEY:
                if event.code in _BTN_CODES and event.value in (0, 1):
                    self._emit(_BTN_CODES[event.code], bool(event.value))
