#!/usr/bin/env python3
"""Monitor field-detector ADC readings and predict robot movement.

This mirrors the current auto_mode sensor filtering and follow controller so we
can tell whether the field detectors are producing sensible inputs before the
robot side is fully trusted.

Input formats accepted on stdin or serial:
  L=123 R=456 IX=789
  123,456,789
  123 456 789
"""

from __future__ import annotations

import argparse
import re
import sys
import time
from dataclasses import dataclass
from typing import Iterable, Iterator, Optional, TextIO


TRACK_ENTRY_SIGNAL = 60
TRACK_EXIT_SIGNAL = 35
TRACK_STARTUP_MIN_FILTERED = 80
INTERSECTION_ENTRY_SIGNAL = 140
INTERSECTION_EXIT_SIGNAL = 90
INTERSECTION_STARTUP_MIN_FILTERED = 140

BASE_SPEED = 600
MAX_PWM = 1000
DEADBAND = 15
KP = 2
FILTER_KEEP_COUNT = 3
BASELINE_IDLE_KEEP_COUNT = 15
BASELINE_STARTUP_KEEP_COUNT = 7
STARTUP_SETTLE_SAMPLES = 16
TURN_SPEED = 500

PATH_STRAIGHT = 0
PATH_LEFT = 1
PATH_RIGHT = 2
PATH_STOP = 3

PATH_TABLE = (
    (PATH_STRAIGHT, PATH_LEFT, PATH_LEFT, PATH_STRAIGHT, PATH_RIGHT, PATH_LEFT, PATH_RIGHT, PATH_STOP),
    (PATH_LEFT, PATH_RIGHT, PATH_LEFT, PATH_RIGHT, PATH_STRAIGHT, PATH_STRAIGHT, PATH_STOP, PATH_STOP),
    (PATH_RIGHT, PATH_STRAIGHT, PATH_RIGHT, PATH_LEFT, PATH_RIGHT, PATH_LEFT, PATH_STRAIGHT, PATH_STOP),
)

LINE_PATTERN = re.compile(
    r"^\s*(?:L\s*=\s*)?(?P<left>-?\d+)[,\s]+(?:R\s*=\s*)?(?P<right>-?\d+)[,\s]+(?:IX\s*=\s*)?(?P<ix>-?\d+)\s*$",
    re.IGNORECASE,
)


@dataclass
class SensorChannel:
    raw: int = 0
    filtered: int = 0
    baseline: int = 0
    signal: int = 0
    detected: bool = False


@dataclass
class FieldData:
    left: SensorChannel
    right: SensorChannel
    intersection: SensorChannel
    samples_seen: int = 0


@dataclass
class PathContext:
    selected_path: int = 0
    intersection_count: int = 0
    intersection_active: bool = False


class AutoModePredictor:
    def __init__(self, path_id: int) -> None:
        self.sensors = FieldData(SensorChannel(), SensorChannel(), SensorChannel())
        self.context = PathContext(selected_path=path_id)
        self.state = "FOLLOW"
        self.active_action = PATH_STRAIGHT

    @staticmethod
    def _mix(previous: int, sample: int, keep_count: int) -> int:
        return (keep_count * previous + sample) // (keep_count + 1)

    @staticmethod
    def _absdiff(a: int, b: int) -> int:
        return a - b if a >= b else b - a

    @staticmethod
    def _baseline_keep_count(samples_seen: int) -> int:
        if samples_seen < STARTUP_SETTLE_SAMPLES:
            return BASELINE_STARTUP_KEEP_COUNT
        return BASELINE_IDLE_KEEP_COUNT

    @staticmethod
    def _detection_active(
        was_detected: bool,
        signal: int,
        filtered: int,
        entry_signal: int,
        exit_signal: int,
        startup_filtered_threshold: int,
    ) -> bool:
        if was_detected:
            return signal >= exit_signal
        return signal >= entry_signal or filtered >= startup_filtered_threshold

    def _update_channel(
        self,
        channel: SensorChannel,
        sample: int,
        entry_signal: int,
        exit_signal: int,
        startup_filtered_threshold: int,
    ) -> None:
        filtered = self._mix(channel.filtered, sample, FILTER_KEEP_COUNT)
        signal = self._absdiff(filtered, channel.baseline)
        detected = self._detection_active(
            channel.detected,
            signal,
            filtered,
            entry_signal,
            exit_signal,
            startup_filtered_threshold,
        )

        if not detected:
            channel.baseline = self._mix(
                channel.baseline,
                filtered,
                self._baseline_keep_count(self.sensors.samples_seen),
            )

        signal = self._absdiff(filtered, channel.baseline)
        detected = self._detection_active(
            channel.detected,
            signal,
            filtered,
            entry_signal,
            exit_signal,
            startup_filtered_threshold,
        )

        channel.raw = sample
        channel.filtered = filtered
        channel.signal = signal
        channel.detected = detected

    @staticmethod
    def _clamp_pwm(value: int) -> int:
        if value < -MAX_PWM:
            return -MAX_PWM
        if value > MAX_PWM:
            return MAX_PWM
        return value

    def _run_follow_controller(self) -> tuple[int, int]:
        error = self.sensors.left.signal - self.sensors.right.signal
        if -DEADBAND < error < DEADBAND:
            error = 0
        correction = KP * error
        left_pwm = self._clamp_pwm(BASE_SPEED - correction)
        right_pwm = self._clamp_pwm(BASE_SPEED + correction)
        return left_pwm, right_pwm

    def _run_path_action(self, action: int) -> tuple[int, int]:
        if action == PATH_LEFT:
            return -TURN_SPEED, TURN_SPEED
        if action == PATH_RIGHT:
            return TURN_SPEED, -TURN_SPEED
        if action == PATH_STOP:
            return 0, 0
        return BASE_SPEED, BASE_SPEED

    def _path_action_name(self, action: int) -> str:
        return {
            PATH_STRAIGHT: "STRAIGHT",
            PATH_LEFT: "LEFT",
            PATH_RIGHT: "RIGHT",
            PATH_STOP: "STOP",
        }[action]

    @staticmethod
    def _movement_label(left_pwm: int, right_pwm: int) -> str:
        if left_pwm == 0 and right_pwm == 0:
            return "STOP"
        if left_pwm > 0 and right_pwm > 0:
            if left_pwm == right_pwm:
                return "FORWARD"
            return "FORWARD_LEFT" if left_pwm < right_pwm else "FORWARD_RIGHT"
        if left_pwm < 0 and right_pwm < 0:
            if left_pwm == right_pwm:
                return "BACKWARD"
            return "BACKWARD_LEFT" if left_pwm > right_pwm else "BACKWARD_RIGHT"
        if left_pwm < 0 and right_pwm > 0:
            return "TURN_LEFT"
        if left_pwm > 0 and right_pwm < 0:
            return "TURN_RIGHT"
        return "MIXED"

    def update(self, left_raw: int, right_raw: int, intersection_raw: int) -> str:
        self._update_channel(
            self.sensors.left,
            left_raw,
            TRACK_ENTRY_SIGNAL,
            TRACK_EXIT_SIGNAL,
            TRACK_STARTUP_MIN_FILTERED,
        )
        self._update_channel(
            self.sensors.right,
            right_raw,
            TRACK_ENTRY_SIGNAL,
            TRACK_EXIT_SIGNAL,
            TRACK_STARTUP_MIN_FILTERED,
        )
        self._update_channel(
            self.sensors.intersection,
            intersection_raw,
            INTERSECTION_ENTRY_SIGNAL,
            INTERSECTION_EXIT_SIGNAL,
            INTERSECTION_STARTUP_MIN_FILTERED,
        )

        left_pwm = 0
        right_pwm = 0

        if self.state == "FOLLOW":
            if not self.sensors.left.detected and not self.sensors.right.detected:
                self.state = "LOST"
                left_pwm, right_pwm = 0, 0
            elif self.sensors.intersection.detected and not self.context.intersection_active:
                self.context.intersection_active = True
                if self.context.intersection_count >= 8:
                    self.active_action = PATH_STOP
                else:
                    self.active_action = PATH_TABLE[self.context.selected_path][self.context.intersection_count]
                    self.context.intersection_count += 1
                self.state = "INTERSECTION"
                left_pwm, right_pwm = self._run_path_action(self.active_action)
            else:
                if not self.sensors.intersection.detected:
                    self.context.intersection_active = False
                left_pwm, right_pwm = self._run_follow_controller()

        elif self.state == "INTERSECTION":
            if not self.sensors.intersection.detected:
                self.context.intersection_active = False
                if self.active_action == PATH_STOP:
                    self.state = "STOP"
                    left_pwm, right_pwm = 0, 0
                else:
                    self.state = "FOLLOW"
                    left_pwm, right_pwm = self._run_follow_controller()
            else:
                left_pwm, right_pwm = self._run_path_action(self.active_action)

        elif self.state == "LOST":
            if self.sensors.left.detected or self.sensors.right.detected:
                self.state = "FOLLOW"
                left_pwm, right_pwm = self._run_follow_controller()
            else:
                left_pwm, right_pwm = 0, 0

        else:
            left_pwm, right_pwm = 0, 0

        self.sensors.samples_seen += 1
        movement = self._movement_label(left_pwm, right_pwm)
        action_name = self._path_action_name(self.active_action)

        return (
            f"L raw={self.sensors.left.raw:4d} filt={self.sensors.left.filtered:4d} "
            f"sig={self.sensors.left.signal:4d} det={int(self.sensors.left.detected)} | "
            f"R raw={self.sensors.right.raw:4d} filt={self.sensors.right.filtered:4d} "
            f"sig={self.sensors.right.signal:4d} det={int(self.sensors.right.detected)} | "
            f"IX raw={self.sensors.intersection.raw:4d} filt={self.sensors.intersection.filtered:4d} "
            f"sig={self.sensors.intersection.signal:4d} det={int(self.sensors.intersection.detected)} | "
            f"state={self.state:<12} action={action_name:<8} "
            f"cmd=({left_pwm:4d},{right_pwm:4d}) movement={movement}"
        )


def parse_adc_line(line: str) -> Optional[tuple[int, int, int]]:
    match = LINE_PATTERN.match(line)
    if not match:
        return None
    return (
        int(match.group("left")),
        int(match.group("right")),
        int(match.group("ix")),
    )


def iter_lines_from_stream(stream: TextIO) -> Iterator[str]:
    for line in stream:
        line = line.strip()
        if line:
            yield line


def iter_lines_from_serial(port: str, baud: int) -> Iterator[str]:
    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise SystemExit("pyserial is required for --port. Install with: pip install pyserial") from exc

    with serial.Serial(port, baud, timeout=1) as ser:
        while True:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            if line:
                yield line


def run_monitor(lines: Iterable[str], path_id: int, sleep_s: float) -> int:
    predictor = AutoModePredictor(path_id=path_id)
    parsed_any = False

    for line in lines:
        values = parse_adc_line(line)
        if values is None:
            print(f"skip: could not parse line: {line}", file=sys.stderr)
            continue
        parsed_any = True
        print(predictor.update(*values))
        if sleep_s > 0:
            time.sleep(sleep_s)

    if parsed_any:
        return 0

    print("No valid ADC lines were parsed.", file=sys.stderr)
    return 1


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Track field-detector ADC readings and print the expected robot movement.",
    )
    parser.add_argument(
        "--port",
        help="Serial port to read from, for example COM8 or /dev/cu.usbserial-xxxx.",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Serial baud rate when --port is used.",
    )
    parser.add_argument(
        "--path-id",
        type=int,
        choices=(1, 2, 3),
        default=1,
        help="Path table to emulate from auto_mode.",
    )
    parser.add_argument(
        "--sleep",
        type=float,
        default=0.0,
        help="Optional delay after each processed sample, useful for replay files.",
    )
    return parser


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    if args.port:
        return run_monitor(iter_lines_from_serial(args.port, args.baud), args.path_id - 1, args.sleep)

    if sys.stdin.isatty():
        print("Paste lines like 'L=123 R=456 IX=789' or pipe a file into this script.", file=sys.stderr)
        return 1

    return run_monitor(iter_lines_from_stream(sys.stdin), args.path_id - 1, args.sleep)


if __name__ == "__main__":
    raise SystemExit(main())
