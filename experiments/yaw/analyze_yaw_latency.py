#!/usr/bin/env python3
"""Analyze C-car yaw latency events without calling telemetry timing a physical delay.

The host timestamps mark callback arrival or CAN enqueue, not motor-internal
sampling or on-wire transmission. Board ticks are useful for per-IMU cadence,
but the two boards' tick origins are not synchronized.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import statistics
from collections import Counter
from dataclasses import dataclass
from pathlib import Path


CURRENT_TO_TORQUE = 33.0 / 2048.0 * 0.07 * 10.0  # LK MG4010Ei10, N m per count
ENCODER_RAD_PER_COUNT = 2.0 * math.pi / 65536.0


@dataclass(frozen=True)
class Event:
    ns: int
    kind: str
    board_tick: int
    v1: float
    v2: float
    v3: float


def load(path: Path) -> dict[str, list[Event]]:
    events: dict[str, list[Event]] = {}
    with path.open(newline="") as file:
        for row in csv.DictReader(file):
            event = Event(
                ns=int(row["host_ns"]),
                kind=row["event"],
                board_tick=int(row["board_tick_quarter_us"]),
                v1=float(row["value1"]),
                v2=float(row["value2"]),
                v3=float(row["value3"]),
            )
            events.setdefault(event.kind, []).append(event)
    for series in events.values():
        series.sort(key=lambda event: event.ns)
    return events


def between(series: list[Event], times: list[int], lo: int, hi: int) -> list[Event]:
    return series[bisect.bisect_left(times, lo) : bisect.bisect_right(times, hi)]


def quantile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    left = int(position)
    return ordered[left] + (ordered[min(left + 1, len(ordered) - 1)] - ordered[left]) * (
        position - left
    )


def describe(values: list[float]) -> dict[str, float | int | None]:
    return {
        "n": len(values),
        "median": statistics.median(values) if values else None,
        "p05": quantile(values, 0.05),
        "p95": quantile(values, 0.95),
    }


def cadence_ms(series: list[Event]) -> dict[str, float | int | None]:
    host = [(b.ns - a.ns) / 1e6 for a, b in zip(series, series[1:]) if b.ns > a.ns]
    board = [
        ((b.board_tick - a.board_tick) & 0xFFFFFFFF) / 4000.0
        for a, b in zip(series, series[1:])
        if a.board_tick and b.board_tick
    ]
    return {"host_arrival_ms": describe(host), "own_board_sample_ms": describe(board)}


def first_crossing(
    series: list[Event], times: list[int], start: int, end: int,
    baseline: float, settled: float, value,
) -> tuple[float | None, float | None]:
    delta = settled - baseline
    if abs(delta) < 0.05:
        return None, None
    threshold = baseline + 0.5 * delta
    after = between(series, times, start, end)
    before = between(series, times, start - 20_000_000, start)
    last_below = before[-1].ns if before else start
    sign = 1.0 if delta > 0 else -1.0
    if before and sign * (value(before[-1]) - threshold) >= 0:
        return None, None
    for item in after:
        if sign * (value(item) - threshold) >= 0:
            return max(0.0, (last_below - start) / 1e6), (item.ns - start) / 1e6
        last_below = item.ns
    return None, None


def current_onset(
    series: list[Event], times: list[int], start: int, end: int,
    direction: float, baseline: float,
) -> tuple[float | None, float | None, float | None]:
    """Bracket the first sustained reported-current change above measured noise."""
    before = between(series, times, start - 20_000_000, start)
    if len(before) < 5:
        return None, None, None
    current = lambda event: event.v1 * CURRENT_TO_TORQUE
    mad = statistics.median(abs(current(item) - baseline) for item in before)
    # Three raw-current counts are about 0.034 N m for this yaw motor.
    threshold = max(3 * CURRENT_TO_TORQUE, 4.0 * 1.4826 * mad)
    if direction * (current(before[-1]) - baseline) >= threshold:
        return None, None, threshold
    after = between(series, times, start, end)
    last_below = before[-1].ns
    for i in range(len(after) - 1):
        if (direction * (current(after[i]) - baseline) >= threshold
                and direction * (current(after[i + 1]) - baseline) >= threshold):
            return (max(0.0, (last_below - start) / 1e6),
                    (after[i].ns - start) / 1e6, threshold)
        last_below = after[i].ns
    return None, None, threshold


def encoder_onset(
    start: int, end: int, direction: float,
    feedback: list[Event], feedback_times: list[int],
) -> tuple[float | None, float | None]:
    # Three encoder counts = 0.0165 degrees. This is a detection threshold,
    # not the instant at which the rotor physically began accelerating.
    before = between(feedback, feedback_times, start - 25_000_000, start - 3_000_000)
    if len(before) < 5:
        return None, None
    origin = round(before[0].v3)

    def wrap_diff(raw: int, anchor: int) -> int:
        return (raw - anchor + 32768) % 65536 - 32768

    center = (origin + round(statistics.median(
        wrap_diff(round(item.v3), origin) for item in before
    ))) % 65536

    def displacement_counts(item: Event) -> float:
        return wrap_diff(round(item.v3), center)

    if max(abs(displacement_counts(item)) for item in before) > 3:
        return None, None
    after = between(feedback, feedback_times, start, end)
    last_below = before[-1].ns
    for i in range(len(after) - 1):
        if (direction * displacement_counts(after[i]) >= 3
                and direction * displacement_counts(after[i + 1]) >= 3):
            return max(0.0, (last_below - start) / 1e6), (after[i].ns - start) / 1e6
        last_below = after[i].ns
    return None, None


def nearest_chassis_z(
    time_ns: int, chassis: list[Event], chassis_times: list[int]
) -> float | None:
    index = bisect.bisect_left(chassis_times, time_ns)
    if index == 0 or index == len(chassis):
        return None
    left, right = chassis[index - 1], chassis[index]
    if time_ns - left.ns > 5_000_000 or right.ns - time_ns > 5_000_000:
        return None
    if right.ns == left.ns:
        return None
    fraction = (time_ns - left.ns) / (right.ns - left.ns)
    return left.v1 + fraction * (right.v1 - left.v1)


def gyro_onset(
    start: int, end: int, direction: float,
    gimbal: list[Event], gimbal_times: list[int],
    chassis: list[Event], chassis_times: list[int],
) -> tuple[float | None, float | None, float | None]:
    # Host-receive alignment is diagnostic only: board clocks are independent.
    samples = []
    for event in between(gimbal, gimbal_times, start - 30_000_000, end):
        chassis_z = nearest_chassis_z(event.ns, chassis, chassis_times)
        if chassis_z is not None:
            samples.append((event.ns, event.v1 - chassis_z))
    baseline = [v for t, v in samples if start - 25_000_000 <= t < start - 3_000_000]
    if len(baseline) < 5:
        return None, None, None
    center = statistics.median(baseline)
    if abs(center) > 0.05:
        return None, None, None
    mad = statistics.median(abs(value - center) for value in baseline)
    threshold = max(0.02, 4.0 * 1.4826 * mad)
    candidates = [(t, v) for t, v in samples if start <= t <= end]
    last_below = max((t for t, _ in samples if t < start), default=start)
    for i in range(len(candidates) - 1):
        t, value = candidates[i]
        if direction * (value - center) >= threshold and direction * (
            candidates[i + 1][1] - center
        ) >= threshold:
            return max(0.0, (last_below - start) / 1e6), (t - start) / 1e6, threshold
        last_below = t
    return None, None, threshold


def analyze(path: Path) -> tuple[dict, list[dict]]:
    series = load(path)
    requests = series.get("request", [])
    enqueues = series.get("can_enqueue", [])
    feedback = series.get("motor_feedback", [])
    gimbal = series.get("gimbal_gyro", [])
    chassis = series.get("chassis_gyro", [])
    if not requests or not enqueues or not feedback:
        raise ValueError("CSV needs request, can_enqueue, and motor_feedback events")
    completed = bool(series.get("test_end") and series["test_end"][-1].v1 == 1.0)
    enqueue_times = [item.ns for item in enqueues]
    feedback_times = [item.ns for item in feedback]
    gimbal_times = [item.ns for item in gimbal]
    chassis_times = [item.ns for item in chassis]

    edges = []
    for index, request in enumerate(requests[1:], start=1):
        if request.v1 == requests[index - 1].v1:
            continue
        next_request = requests[index + 1].ns if index + 1 < len(requests) else request.ns + 50_000_000
        matches = [
            event for event in between(enqueues, enqueue_times, request.ns, next_request)
            if math.isfinite(event.v1) and abs(event.v1 - request.v1) < 0.001
        ]
        if not matches:
            continue
        enqueue = matches[0]
        pre = between(feedback, feedback_times, enqueue.ns - 20_000_000, enqueue.ns - 2_000_000)
        post = between(feedback, feedback_times, enqueue.ns + 8_000_000,
                       min(enqueue.ns + 25_000_000, next_request))
        baseline = statistics.median(item.v1 * CURRENT_TO_TORQUE for item in pre) if len(pre) >= 3 else None
        settled = statistics.median(item.v1 * CURRENT_TO_TORQUE for item in post) if len(post) >= 3 else None
        current_lo = current_hi = None
        onset_lo = onset_hi = onset_threshold = None
        if baseline is not None and settled is not None:
            current_lo, current_hi = first_crossing(
                feedback, feedback_times, enqueue.ns, next_request,
                baseline, settled, lambda item: item.v1 * CURRENT_TO_TORQUE,
            )
            if request.v1 != 0.0:
                onset_lo, onset_hi, onset_threshold = current_onset(
                    feedback, feedback_times, enqueue.ns,
                    min(enqueue.ns + 25_000_000, next_request),
                    math.copysign(1.0, settled - baseline), baseline,
                )
        gyro_lo = gyro_hi = gyro_threshold = None
        encoder_lo = encoder_hi = None
        if request.v1 != 0.0 and gimbal and chassis:
            gyro_lo, gyro_hi, gyro_threshold = gyro_onset(
                enqueue.ns, min(enqueue.ns + 35_000_000, next_request),
                math.copysign(1.0, request.v1), gimbal, gimbal_times,
                chassis, chassis_times,
            )
        if request.v1 != 0.0:
            encoder_lo, encoder_hi = encoder_onset(
                enqueue.ns, min(enqueue.ns + 35_000_000, next_request),
                math.copysign(1.0, request.v1), feedback, feedback_times,
            )
        edges.append({
            "request_torque_nm": request.v1,
            "request_to_enqueue_ms": (enqueue.ns - request.ns) / 1e6,
            "reported_current_onset_lower_ms": onset_lo,
            "reported_current_onset_upper_ms": onset_hi,
            "current_onset_threshold_nm": onset_threshold,
            "reported_current_50pct_lower_ms": current_lo,
            "reported_current_50pct_upper_ms": current_hi,
            "first_detectable_gyro_lower_ms": gyro_lo,
            "first_detectable_gyro_upper_ms": gyro_hi,
            "gyro_threshold_rad_s": gyro_threshold,
            "first_detectable_encoder_lower_ms": encoder_lo,
            "first_detectable_encoder_upper_ms": encoder_hi,
            "feedback_before_n": len(pre),
            "feedback_after_n": len(post),
        })

    summary = {
        "source": str(path),
        "completed": completed,
        "meaning": {
            "current_interval": "CAN enqueue on host -> first motor-reported current crossing 50%; includes transport and telemetry lag",
            "gyro_interval": "CAN enqueue on host -> first detectable relative gyro change; host-aligned, threshold-dependent diagnostic",
            "encoder_interval": "CAN enqueue on host -> first persistent yaw encoder change >= 3 counts (%.6g rad); threshold-dependent diagnostic" % (3 * ENCODER_RAD_PER_COUNT),
            "physical_delay": "not identifiable from this CSV",
        },
        "event_counts": dict(Counter({key: len(items) for key, items in series.items()})),
        "cadence": {
            key: cadence_ms(series.get(key, []))
            for key in ("motor_feedback", "gimbal_gyro", "chassis_gyro")
        },
        "edges_total": len(edges),
        "request_to_enqueue_ms": describe([e["request_to_enqueue_ms"] for e in edges]),
        "reported_current_onset_lower_ms": describe([
            e["reported_current_onset_lower_ms"] for e in edges
            if e["reported_current_onset_lower_ms"] is not None
        ]),
        "reported_current_onset_upper_ms": describe([
            e["reported_current_onset_upper_ms"] for e in edges
            if e["reported_current_onset_upper_ms"] is not None
        ]),
        "reported_current_50pct_lower_ms": describe([
            e["reported_current_50pct_lower_ms"] for e in edges
            if e["reported_current_50pct_lower_ms"] is not None
        ]),
        "reported_current_50pct_upper_ms": describe([
            e["reported_current_50pct_upper_ms"] for e in edges
            if e["reported_current_50pct_upper_ms"] is not None
        ]),
        "reported_current_bracket_width_ms": describe([
            e["reported_current_50pct_upper_ms"] - e["reported_current_50pct_lower_ms"]
            for e in edges if e["reported_current_50pct_upper_ms"] is not None
            and e["reported_current_50pct_lower_ms"] is not None
        ]),
        "first_detectable_gyro_upper_ms": describe([
            e["first_detectable_gyro_upper_ms"] for e in edges
            if e["first_detectable_gyro_upper_ms"] is not None
        ]),
        "first_detectable_encoder_upper_ms": describe([
            e["first_detectable_encoder_upper_ms"] for e in edges
            if e["first_detectable_encoder_upper_ms"] is not None
        ]),
        "torque_step_groups": {
            label: {
                "edges": len(group),
                "reported_current_onset_lower_ms": describe([
                    e["reported_current_onset_lower_ms"] for e in group
                    if e["reported_current_onset_lower_ms"] is not None
                ]),
                "reported_current_onset_upper_ms": describe([
                    e["reported_current_onset_upper_ms"] for e in group
                    if e["reported_current_onset_upper_ms"] is not None
                ]),
                "reported_current_50pct_lower_ms": describe([
                    e["reported_current_50pct_lower_ms"] for e in group
                    if e["reported_current_50pct_lower_ms"] is not None
                ]),
                "reported_current_50pct_upper_ms": describe([
                    e["reported_current_50pct_upper_ms"] for e in group
                    if e["reported_current_50pct_upper_ms"] is not None
                ]),
            }
            for label, group in (
                ("positive_on", [e for e in edges if e["request_torque_nm"] > 0]),
                ("negative_on", [e for e in edges if e["request_torque_nm"] < 0]),
                ("off", [e for e in edges if e["request_torque_nm"] == 0]),
            )
        },
        "notes": [
            "Incomplete or emergency-stopped runs should not be used as full-test summaries.",
            "All host times are monotonic-clock callback/queue times, not board CAN wire times.",
            "The motor feedback CAN frame has no board sample timestamp in the current SDK.",
            "Board tick units are 0.25 us, but IMU samples occur much less often; tick precision is not delay accuracy.",
            "The top and bottom board tick origins are independent; never subtract them directly.",
            "A dropped-event count is printed in the robot log when the test ends; discard overloaded recordings.",
        ],
    }
    return summary, edges


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, help="yaw_latency_*.csv copied from the robot")
    args = parser.parse_args()
    summary, edges = analyze(args.csv)
    summary_path = args.csv.with_suffix(".summary.json")
    edges_path = args.csv.with_suffix(".edges.csv")
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
    with edges_path.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(edges[0]) if edges else ["request_torque_nm"])
        writer.writeheader()
        writer.writerows(edges)
    print(f"Summary: {summary_path}\nPer-edge intervals: {edges_path}")
    print(json.dumps({key: summary[key] for key in (
        "edges_total", "request_to_enqueue_ms", "reported_current_onset_lower_ms",
        "reported_current_onset_upper_ms", "torque_step_groups", "reported_current_50pct_lower_ms",
        "reported_current_50pct_upper_ms", "reported_current_bracket_width_ms",
        "first_detectable_gyro_upper_ms", "first_detectable_encoder_upper_ms")}, indent=2))


if __name__ == "__main__":
    main()
