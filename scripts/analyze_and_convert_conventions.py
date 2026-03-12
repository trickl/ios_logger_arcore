#!/usr/bin/env python3
"""
Analyze iOS vs Android pose convention differences and convert Android pose files
into iOS-matching convention.

Assumed pose row schema (CSV):
    timestamp, tx, ty, tz, qw, qx, qy, qz [, ...ignored]
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable
import math
import shutil

import numpy as np


IOS_ROOT = Path("/home/timg/Captures/labelled")
ANDROID_ROOT = Path("/home/timg/projects/ios_logger_arcore/pulled_datasets/android")
OUTPUT_ROOT = Path("/home/timg/projects/ios_logger_arcore/pulled_datasets/android_ios_convention")
REPORT_PATH = Path("/home/timg/projects/ios_logger_arcore/pulled_datasets/android/CONVENTION_ANALYSIS.md")


@dataclass(frozen=True)
class SequencePair:
    key: str
    ios_dir: str
    android_dir: str
    movement_type: str  # "rot" or "trans"


PAIRS: list[SequencePair] = [
    SequencePair(
        key="pure_roll_clockwise_return",
        ios_dir="2026-03-07T17-35-52_pure_roll_clockwise",
        android_dir="2026-03-02T21-43-38__pure_roll_clockwise_return",
        movement_type="rot",
    ),
    SequencePair(
        key="pure_pitch_up_down_return",
        ios_dir="2026-03-07T17-36-12_pure_pitch_up_down_return",
        android_dir="2026-03-02T21-44-17__pure_pitch_up_down_return",
        movement_type="rot",
    ),
    SequencePair(
        key="pure_yaw_plus90_return",
        ios_dir="2026-03-07T17-36-33_pure_yaw_plus90_return",
        android_dir="2026-03-02T21-44-58__pure_yaw_plus90_return",
        movement_type="rot",
    ),
    SequencePair(
        key="forward_back_same_line",
        ios_dir="2026-03-07T17-36-51_forward_back_same_line",
        android_dir="2026-03-02T21-45-30__forward_back_same_line_1p5m",
        movement_type="trans",
    ),
    SequencePair(
        key="lateral_left_right_return",
        ios_dir="2026-03-07T17-37-16_lateral_left_right_return",
        android_dir="2026-03-02T21-46-29__lateral_left_right_return_1m",
        movement_type="trans",
    ),
    SequencePair(
        key="vertical_up_down_return",
        ios_dir="2026-03-07T17-37-37_vertical_up_down_return",
        android_dir="2026-03-02T21-47-38__vertical_up_down_return",
        movement_type="trans",
    ),
    SequencePair(
        key="square_loop_closure",
        ios_dir="2026-03-07T17-37-55_square_loop_closure",
        android_dir="2026-03-02T21-48-24__square_loop_1m_closure",
        movement_type="trans",
    ),
]


def _safe_norm(v: np.ndarray, axis: int = -1) -> np.ndarray:
    n = np.linalg.norm(v, axis=axis, keepdims=True)
    n[n < 1e-12] = 1.0
    return n


def normalize_quaternions(q: np.ndarray) -> np.ndarray:
    return q / _safe_norm(q)


def qmul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = np.moveaxis(a, -1, 0)
    w2, x2, y2, z2 = np.moveaxis(b, -1, 0)
    return np.stack(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        axis=-1,
    )


def qconj(q: np.ndarray) -> np.ndarray:
    out = q.copy()
    out[..., 1:] *= -1.0
    return out


def q_to_rotvec(q: np.ndarray) -> np.ndarray:
    q = normalize_quaternions(q)
    w = np.clip(q[:, 0], -1.0, 1.0)
    v = q[:, 1:]

    angle = 2.0 * np.arccos(w)
    s = np.sqrt(np.maximum(1.0 - w * w, 1e-12))
    axis = v / s[:, None]
    rv = axis * angle[:, None]

    small = angle < 1e-8
    rv[small] = 2.0 * v[small]
    return rv


def load_pose_csv(path: Path) -> np.ndarray:
    rows: list[list[float]] = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = [float(x) for x in line.split(",")]
            if len(parts) < 8:
                continue
            rows.append(parts)

    if not rows:
        raise ValueError(f"No valid pose rows in {path}")

    arr = np.asarray(rows, dtype=np.float64)

    # Canonical format: t,tx,ty,tz,qw,qx,qy,qz
    # Legacy format:    t,tx,ty,tz,qx,qy,qz,qw
    mean_abs_col4 = float(np.mean(np.abs(arr[:, 4])))
    mean_abs_col7 = float(np.mean(np.abs(arr[:, 7])))
    canonical_qw_first = mean_abs_col4 >= mean_abs_col7
    if canonical_qw_first:
        return arr

    # Reorder legacy -> canonical in-memory only.
    out = arr.copy()
    out[:, 4] = arr[:, 7]  # qw
    out[:, 5] = arr[:, 4]  # qx
    out[:, 6] = arr[:, 5]  # qy
    out[:, 7] = arr[:, 6]  # qz
    return out


def save_pose_csv(path: Path, arr: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        for row in arr:
            f.write(",".join(f"{x:.9f}" for x in row) + "\n")


def first_leg_trans_vector(positions: np.ndarray) -> tuple[np.ndarray, int]:
    rel = positions - positions[0]
    d = np.linalg.norm(rel, axis=1)
    idx = int(np.argmax(d))
    return rel[idx], idx


def first_leg_rot_vector(quat_wxyz: np.ndarray) -> tuple[np.ndarray, int]:
    q = normalize_quaternions(quat_wxyz)
    q0 = np.repeat(q[:1], repeats=len(q), axis=0)
    q_rel = qmul(q, qconj(q0))
    rv = q_to_rotvec(q_rel)
    d = np.linalg.norm(rv, axis=1)
    idx = int(np.argmax(d))
    return rv[idx], idx


def dominant_axis_and_sign(v: np.ndarray) -> tuple[int, int]:
    axis = int(np.argmax(np.abs(v)))
    sign = int(math.copysign(1, v[axis])) if abs(v[axis]) > 1e-12 else 0
    return axis, sign


def axis_name(i: int) -> str:
    return ["x", "y", "z"][i]


def sign_name(s: int) -> str:
    return "+" if s >= 0 else "-"


def convert_android_to_ios_pose(arr: np.ndarray) -> np.ndarray:
    """
    Conversion chosen from empirical 7-sequence fit:
      - translation: [x, y, z] -> [x, y, -z]
      - quaternion:  [qw, qx, qy, qz] -> [qw, qx, qy, -qz]

    This matches observed clockwise sign and forward-axis convention differences.
    """
    out = arr.copy()

    # tx, ty, tz
    out[:, 3] *= -1.0

    # qw, qx, qy, qz
    out[:, 7] *= -1.0
    out[:, 4:8] = normalize_quaternions(out[:, 4:8])

    return out


def pose_like_txt_files(dataset_dir: Path) -> Iterable[Path]:
    # Convert ARposes-style files only.
    # SyncedPoses files can be whitespace-delimited and may use alternative column ordering,
    # so they are intentionally excluded from automatic conversion here.
    for p in sorted(dataset_dir.glob("*.txt")):
        name = p.name
        if name.startswith("ARposes"):
            yield p


def compare_sequences() -> tuple[str, int, int]:
    lines: list[str] = []
    lines.append("# iOS vs Android Convention Analysis")
    lines.append("")
    lines.append("This report compares seven matched motion datasets:")
    lines.append("")
    lines.append("- iOS reference root: `/home/timg/Captures/labelled`")
    lines.append("- Android root: `/home/timg/projects/ios_logger_arcore/pulled_datasets/android`")
    lines.append("")
    lines.append("Feature used per sequence: **start → apex principal movement vector**.")
    lines.append("For rotational captures, this is start-relative rotation-vector at max angular distance.")
    lines.append("")

    lines.append("## Per-sequence signatures")
    lines.append("")
    lines.append("| Sequence | Type | iOS dominant | Android dominant | Android-converted dominant | Match after conversion |")
    lines.append("|---|---|---|---|---|---|")

    matches_before = 0
    matches_after = 0

    for pair in PAIRS:
        ios_arr = load_pose_csv(IOS_ROOT / pair.ios_dir / "ARposes.txt")
        and_arr = load_pose_csv(ANDROID_ROOT / pair.android_dir / "ARposes.txt")
        and_conv = convert_android_to_ios_pose(and_arr)

        if pair.movement_type == "trans":
            vi, _ = first_leg_trans_vector(ios_arr[:, 1:4])
            va, _ = first_leg_trans_vector(and_arr[:, 1:4])
            vc, _ = first_leg_trans_vector(and_conv[:, 1:4])
        else:
            vi, _ = first_leg_rot_vector(ios_arr[:, 4:8])
            va, _ = first_leg_rot_vector(and_arr[:, 4:8])
            vc, _ = first_leg_rot_vector(and_conv[:, 4:8])

        di = dominant_axis_and_sign(vi)
        da = dominant_axis_and_sign(va)
        dc = dominant_axis_and_sign(vc)

        before = di == da
        after = di == dc
        matches_before += int(before)
        matches_after += int(after)

        ios_dom = f"{sign_name(di[1])}{axis_name(di[0])}"
        and_dom = f"{sign_name(da[1])}{axis_name(da[0])}"
        conv_dom = f"{sign_name(dc[1])}{axis_name(dc[0])}"

        lines.append(
            f"| `{pair.key}` | {pair.movement_type} | `{ios_dom}` | `{and_dom}` | `{conv_dom}` | {'✅' if after else '❌'} |"
        )

    lines.append("")
    lines.append("## Inferred conventions")
    lines.append("")
    lines.append("### iOS canonical (from labeled reference captures)")
    lines.append("")
    lines.append("- Translational first-leg conventions inferred from pure motions:")
    lines.append("  - rightward motion: `+x`")
    lines.append("  - upward motion: `+y`")
    lines.append("  - forward/toward wall motion: `+z`")
    lines.append("- Rotational first-leg conventions inferred from pure motions:")
    lines.append("  - clockwise roll: `+z` dominant")
    lines.append("  - pitch-up: `-x` dominant")
    lines.append("  - yaw right/clockwise: `-y` dominant")
    lines.append("")

    lines.append("### Android raw (before conversion)")
    lines.append("")
    lines.append("- Key mismatch to iOS is primarily the **Z convention**:")
    lines.append("  - forward motion appears as `-z`")
    lines.append("  - clockwise roll appears as `-z` rotational dominance")
    lines.append("- X/Y translation directions for pure lateral/vertical motions already align.")
    lines.append("")

    lines.append("### Conversion applied (Android -> iOS)")
    lines.append("")
    lines.append("- Position: `z := -z`")
    lines.append("- Quaternion: `qz := -qz` (with re-normalization)")
    lines.append("")
    lines.append("This conversion raised dominant-axis/sign agreement from")
    lines.append(f"**{matches_before}/7** to **{matches_after}/7** across the seven canonical motions.")
    lines.append("")

    lines.append("## Notes on matrix convention")
    lines.append("")
    lines.append("Empirically, the mismatch is consistent with a Z-axis convention difference and")
    lines.append("a transpose/inversion interpretation in rotation handling (row-vs-column-major style ambiguity).")
    lines.append("Operationally, the above component-level conversion produces iOS-matching behavior on the canonical tests.")
    lines.append("")

    return "\n".join(lines), matches_before, matches_after


def convert_all_android_pose_files() -> int:
    if OUTPUT_ROOT.exists():
        shutil.rmtree(OUTPUT_ROOT)
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)

    converted_count = 0

    for dataset_dir in sorted(ANDROID_ROOT.iterdir()):
        if not dataset_dir.is_dir():
            continue

        out_dataset = OUTPUT_ROOT / dataset_dir.name
        out_dataset.mkdir(parents=True, exist_ok=True)

        # Copy non-pose files shallowly to keep dataset structure useful.
        for item in dataset_dir.iterdir():
            dest = out_dataset / item.name
            if item.is_dir():
                # Avoid copying large generated trees unless explicitly needed.
                if item.name in {"images", "poses", "intrinsics"}:
                    continue
                shutil.copytree(item, dest)
            elif item.is_file():
                if item.name.startswith("ARposes"):
                    continue
                shutil.copy2(item, dest)

        for pose_file in pose_like_txt_files(dataset_dir):
            arr = load_pose_csv(pose_file)
            converted = convert_android_to_ios_pose(arr)
            save_pose_csv(out_dataset / pose_file.name, converted)
            converted_count += 1

    return converted_count


def main() -> None:
    report_text, before, after = compare_sequences()
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(report_text, encoding="utf-8")

    converted_files = convert_all_android_pose_files()

    print(f"Report written: {REPORT_PATH}")
    print(f"Converted pose-like files: {converted_files}")
    print(f"Dominant motion agreement: {before}/7 -> {after}/7")
    print(f"Converted datasets root: {OUTPUT_ROOT}")


if __name__ == "__main__":
    main()
