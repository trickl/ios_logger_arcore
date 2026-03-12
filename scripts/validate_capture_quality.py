#!/usr/bin/env python3
"""
Validate capture robustness/synchronization quality for ios_logger_arcore datasets.

Checks performed:
- required artifacts exist
- ARposes timestamps strictly increasing
- pose/frame count synchronization
- pause/resume interval consistency using CaptureEvents timestamps
- presence of first-frame-relative reference event for new pipeline
- summary of relocalization/rejection/hold events
- per-step translation/rotation stats in exported pose trajectory

Usage:
  python3 scripts/validate_capture_quality.py --dataset pulled_datasets/<capture_dir>
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Tuple


REQUIRED_FILES = [
    "Frames.m4v",
    "Frames.txt",
    "ARposes.txt",
    "CaptureEvents.txt",
    "Accel.txt",
    "Gyro.txt",
    "GPS.txt",
    "Head.txt",
    "Motion.txt",
    "MotARH.txt",
    "MotMagnFull.txt",
    "Magnet.txt",
]


@dataclass
class PoseRow:
    t: float
    tx: float
    ty: float
    tz: float
    qw: float
    qx: float
    qy: float
    qz: float


@dataclass
class EventRow:
    t: float
    name: str
    details: str


@dataclass
class CheckResult:
    ok: bool
    label: str
    details: str


def _iter_csv_rows(path: Path) -> Iterable[List[str]]:
    with path.open("r", encoding="utf-8") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue
            yield [c.strip() for c in row]


def load_poses(path: Path) -> List[PoseRow]:
    parsed: List[List[float]] = []
    for row in _iter_csv_rows(path):
        if len(row) < 8:
            continue
        try:
            parsed.append([float(x) for x in row[:8]])
        except ValueError:
            continue

    poses: List[PoseRow] = []
    if not parsed:
        return poses

    # Canonical ios_logger format is: t,tx,ty,tz,qw,qx,qy,qz
    # Older Android captures used:      t,tx,ty,tz,qx,qy,qz,qw
    # Heuristic fallback: w component usually has larger |value| distribution.
    mean_abs_col4 = sum(abs(v[4]) for v in parsed) / len(parsed)
    mean_abs_col7 = sum(abs(v[7]) for v in parsed) / len(parsed)
    canonical_qw_first = mean_abs_col4 >= mean_abs_col7

    for vals in parsed:
        if canonical_qw_first:
            qw, qx, qy, qz = vals[4], vals[5], vals[6], vals[7]
        else:
            qx, qy, qz, qw = vals[4], vals[5], vals[6], vals[7]

        poses.append(
            PoseRow(
                t=vals[0],
                tx=vals[1],
                ty=vals[2],
                tz=vals[3],
                qw=qw,
                qx=qx,
                qy=qy,
                qz=qz,
            )
        )
    return poses


def load_frame_timestamps(path: Path) -> List[float]:
    out: List[float] = []
    for row in _iter_csv_rows(path):
        if len(row) < 2:
            continue
        try:
            out.append(float(row[0]))
        except ValueError:
            continue
    return out


def load_events(path: Path) -> List[EventRow]:
    events: List[EventRow] = []
    for row in _iter_csv_rows(path):
        if len(row) < 2:
            continue
        try:
            ts = float(row[0])
        except ValueError:
            continue
        name = row[1]
        details = row[2] if len(row) > 2 else ""
        events.append(EventRow(t=ts, name=name, details=details))
    return events


def quaternion_angle_degrees(a: PoseRow, b: PoseRow) -> float:
    aw, ax, ay, az = a.qw, a.qx, a.qy, a.qz
    bw, bx, by, bz = b.qw, b.qx, b.qy, b.qz

    an = math.sqrt(aw * aw + ax * ax + ay * ay + az * az)
    bn = math.sqrt(bw * bw + bx * bx + by * by + bz * bz)
    if an <= 1e-12 or bn <= 1e-12:
        return 0.0

    aw, ax, ay, az = aw / an, ax / an, ay / an, az / an
    bw, bx, by, bz = bw / bn, bx / bn, by / bn, bz / bn

    dot = abs(aw * bw + ax * bx + ay * by + az * bz)
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


def check_required_files(dataset: Path) -> CheckResult:
    missing = [name for name in REQUIRED_FILES if not (dataset / name).exists()]
    if missing:
        return CheckResult(False, "required_files", f"missing={missing}")
    return CheckResult(True, "required_files", "all required files present")


def check_pose_timestamps_strict(poses: List[PoseRow]) -> CheckResult:
    if not poses:
        return CheckResult(False, "pose_timestamps", "no pose rows parsed")
    non_increasing = 0
    equal_neighbors = 0
    for i in range(1, len(poses)):
        if poses[i].t <= poses[i - 1].t:
            non_increasing += 1
            if poses[i].t == poses[i - 1].t:
                equal_neighbors += 1
    if non_increasing > 0:
        return CheckResult(
            False,
            "pose_timestamps",
            f"non_increasing={non_increasing},equal_neighbors={equal_neighbors},rows={len(poses)}",
        )
    return CheckResult(True, "pose_timestamps", f"strictly increasing rows={len(poses)}")


def check_frame_pose_sync(frames_ts: List[float], poses: List[PoseRow]) -> CheckResult:
    if not frames_ts and not poses:
        return CheckResult(False, "frame_pose_sync", "no frame and pose rows")
    if len(frames_ts) != len(poses):
        return CheckResult(False, "frame_pose_sync", f"frames={len(frames_ts)} poses={len(poses)}")
    return CheckResult(True, "frame_pose_sync", f"frames==poses=={len(poses)}")


def check_reference_event(events: List[EventRow]) -> CheckResult:
    engine_started = [e for e in events if e.name == "engine_started"]
    requires_reference_event = any("first_frame_relative=true" in e.details for e in engine_started)

    found = any(e.name == "first_frame_reference_set" for e in events)
    if not found and requires_reference_event:
        return CheckResult(False, "first_frame_reference", "missing first_frame_reference_set event")
    if not found and not requires_reference_event:
        return CheckResult(
            True,
            "first_frame_reference",
            "not required for legacy dataset (engine metadata lacks first_frame_relative=true)",
        )
    return CheckResult(True, "first_frame_reference", "first-frame-relative reference event found")


def build_pause_intervals(events: List[EventRow]) -> Tuple[List[Tuple[float, float]], Optional[str]]:
    intervals: List[Tuple[float, float]] = []
    active_pause_start: Optional[float] = None
    for e in events:
        if e.name == "capture_paused":
            if active_pause_start is None:
                active_pause_start = e.t
        elif e.name == "capture_resumed":
            if active_pause_start is not None:
                intervals.append((active_pause_start, e.t))
                active_pause_start = None
    if active_pause_start is not None:
        return intervals, f"unclosed:{active_pause_start:.6f}"
    return intervals, None


def check_pause_intervals_have_no_writes(
    intervals: List[Tuple[float, float]],
    frame_ts: List[float],
    pose_ts: List[float],
) -> CheckResult:
    if not intervals:
        return CheckResult(True, "pause_intervals", "no pause intervals found")

    violations = 0
    eps = 1e-6
    for start, end in intervals:
        # Use open interval: writes at exact boundary timestamps can coincide with
        # pause/resume event logging order and are not leakage.
        frame_hits = sum(1 for t in frame_ts if (start + eps) < t < (end - eps))
        pose_hits = sum(1 for t in pose_ts if (start + eps) < t < (end - eps))
        if frame_hits > 0 or pose_hits > 0:
            violations += 1

    if violations > 0:
        return CheckResult(False, "pause_intervals", f"intervals={len(intervals)} violations={violations}")
    return CheckResult(True, "pause_intervals", f"intervals={len(intervals)} no writes during pauses")


def summarize_event_counts(events: List[EventRow]) -> str:
    interesting = {
        "tracking_relocalized",
        "pose_rejected_discontinuity",
        "pose_hold_relocalization",
        "capture_paused",
        "capture_resumed",
        "tracking_fsm_transition",
    }
    counts = {}
    for e in events:
        if e.name in interesting:
            counts[e.name] = counts.get(e.name, 0) + 1
    if not counts:
        return "none"
    return ", ".join(f"{k}={counts[k]}" for k in sorted(counts))


def motion_stats(poses: List[PoseRow]) -> str:
    if len(poses) < 2:
        return "insufficient rows"
    trans = []
    rot = []
    for i in range(1, len(poses)):
        a = poses[i - 1]
        b = poses[i]
        dx = b.tx - a.tx
        dy = b.ty - a.ty
        dz = b.tz - a.tz
        trans.append(math.sqrt(dx * dx + dy * dy + dz * dz))
        rot.append(quaternion_angle_degrees(a, b))

    def p95(vals: List[float]) -> float:
        if not vals:
            return 0.0
        s = sorted(vals)
        idx = min(len(s) - 1, int(round(0.95 * (len(s) - 1))))
        return s[idx]

    return (
        f"step_translation_m: mean={sum(trans)/len(trans):.4f}, max={max(trans):.4f}, p95={p95(trans):.4f}; "
        f"step_rotation_deg: mean={sum(rot)/len(rot):.3f}, max={max(rot):.3f}, p95={p95(rot):.3f}"
    )


def main() -> None:
    ap = argparse.ArgumentParser(description="Validate robust capture quality for ios_logger_arcore datasets")
    ap.add_argument("--dataset", required=True, type=Path, help="Path to dataset directory")
    args = ap.parse_args()

    dataset = args.dataset
    if not dataset.exists() or not dataset.is_dir():
        raise SystemExit(f"Dataset directory does not exist: {dataset}")

    results: List[CheckResult] = []
    results.append(check_required_files(dataset))

    poses = load_poses(dataset / "ARposes.txt") if (dataset / "ARposes.txt").exists() else []
    frames_ts = load_frame_timestamps(dataset / "Frames.txt") if (dataset / "Frames.txt").exists() else []
    events = load_events(dataset / "CaptureEvents.txt") if (dataset / "CaptureEvents.txt").exists() else []

    results.append(check_pose_timestamps_strict(poses))
    results.append(check_frame_pose_sync(frames_ts, poses))
    results.append(check_reference_event(events))

    intervals, interval_warning = build_pause_intervals(events)
    if interval_warning and interval_warning.startswith("unclosed:"):
        last_pause = float(interval_warning.split(":", 1)[1])
        eps = 1e-6
        writes_after = (
            sum(1 for t in frames_ts if t > last_pause + eps) +
            sum(1 for p in poses if p.t > last_pause + eps)
        )
        if writes_after == 0:
            results.append(
                CheckResult(
                    True,
                    "pause_intervals",
                    "capture ended while paused (no writes after last pause)",
                )
            )
        else:
            results.append(
                CheckResult(
                    False,
                    "pause_intervals",
                    f"unclosed pause interval with trailing writes={writes_after}",
                )
            )
    elif interval_warning:
        results.append(CheckResult(False, "pause_intervals", interval_warning))
    else:
        results.append(
            check_pause_intervals_have_no_writes(intervals, frames_ts, [p.t for p in poses])
        )

    print(f"dataset: {dataset}")
    print("\nchecks:")
    for r in results:
        mark = "PASS" if r.ok else "FAIL"
        print(f"  [{mark}] {r.label}: {r.details}")

    print("\nsummary:")
    print(f"  poses: {len(poses)}")
    print(f"  frames: {len(frames_ts)}")
    print(f"  events: {len(events)}")
    print(f"  event_counts: {summarize_event_counts(events)}")
    print(f"  motion: {motion_stats(poses)}")

    failed = [r for r in results if not r.ok]
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
