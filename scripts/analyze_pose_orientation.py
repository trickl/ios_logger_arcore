#!/usr/bin/env python3
"""
Analyze trajectory orientation / planarity / loop-closure from AR pose logs.

Input format (default ios_logger_arcore):
  timestamp,tx,ty,tz,qw,qx,qy,qz

This script is intentionally translation-centric, because "single-plane square loop"
claims were based on camera centers (tx,ty,tz), not quaternion interpretation.

Examples:
  python scripts/analyze_pose_orientation.py \
      --poses pulled_datasets/2026-03-02T21-48-24/ARposes_legacy.txt

  # Test a convention hypothesis by remapping axes (output X,Y,Z from input axes):
  # e.g. swap Y/Z and flip new Z
  python scripts/analyze_pose_orientation.py \
      --poses pulled_datasets/.../ARposes.txt \
      --axis-map x,z,-y
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple

import numpy as np


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


def _parse_line_flexible(line: str) -> List[float]:
    s = line.strip()
    if not s:
        return []
    if "," in s:
        parts = [p.strip() for p in s.split(",")]
    else:
        parts = s.split()
    if len(parts) < 8:
        return []
    try:
        return [float(x) for x in parts[:8]]
    except ValueError:
        return []


def load_poses(path: Path) -> List[PoseRow]:
    parsed: List[List[float]] = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            vals = _parse_line_flexible(line)
            if not vals:
                continue
            parsed.append(vals)

    rows: List[PoseRow] = []
    if not parsed:
        return rows

    # Canonical format: t,tx,ty,tz,qw,qx,qy,qz
    # Legacy format:    t,tx,ty,tz,qx,qy,qz,qw
    mean_abs_col4 = sum(abs(v[4]) for v in parsed) / len(parsed)
    mean_abs_col7 = sum(abs(v[7]) for v in parsed) / len(parsed)
    canonical_qw_first = mean_abs_col4 >= mean_abs_col7

    for vals in parsed:
        if canonical_qw_first:
            qw, qx, qy, qz = vals[4], vals[5], vals[6], vals[7]
        else:
            qx, qy, qz, qw = vals[4], vals[5], vals[6], vals[7]
        rows.append(PoseRow(vals[0], vals[1], vals[2], vals[3], qw, qx, qy, qz))

    return rows


def parse_axis_map(spec: str) -> Tuple[Tuple[int, float], Tuple[int, float], Tuple[int, float]]:
    # output axes X,Y,Z each expressed as +/- input axis x|y|z
    parts = [p.strip().lower() for p in spec.split(",")]
    if len(parts) != 3:
        raise ValueError("--axis-map must have three comma-separated entries, e.g. x,y,z or x,z,-y")

    def decode(token: str) -> Tuple[int, float]:
        sign = 1.0
        if token.startswith("-"):
            sign = -1.0
            token = token[1:]
        elif token.startswith("+"):
            token = token[1:]

        m = {"x": 0, "y": 1, "z": 2}
        if token not in m:
            raise ValueError(f"Invalid axis token '{token}'. Use x/y/z with optional sign.")
        return m[token], sign

    out = tuple(decode(p) for p in parts)
    used = sorted(idx for idx, _ in out)
    if used != [0, 1, 2]:
        raise ValueError("--axis-map must use each input axis exactly once.")
    return out  # type: ignore[return-value]


def remap_points(points_xyz: np.ndarray, axis_map: Tuple[Tuple[int, float], Tuple[int, float], Tuple[int, float]]) -> np.ndarray:
    out = np.zeros_like(points_xyz)
    for out_i, (in_i, sign) in enumerate(axis_map):
        out[:, out_i] = sign * points_xyz[:, in_i]
    return out


def quat_wxyz_to_rot(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
    n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if n <= 1e-12:
        qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0
    else:
        qw, qx, qy, qz = qw / n, qx / n, qy / n, qz / n

    # Rotation matrix for quaternion [w, x, y, z]
    return np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ],
        dtype=np.float64,
    )


def extract_points(rows: List[PoseRow], pose_semantics: str) -> np.ndarray:
    pts = []
    for r in rows:
        t = np.array([r.tx, r.ty, r.tz], dtype=np.float64)
        if pose_semantics == "camera_center":
            c = t
        elif pose_semantics == "world_to_camera_t":
            # If pose is x_cam = R * x_world + t, camera center in world is C = -R^T t.
            rwc = quat_wxyz_to_rot(r.qw, r.qx, r.qy, r.qz)
            c = -(rwc.T @ t)
        else:
            raise ValueError(f"Unsupported pose semantics: {pose_semantics}")
        pts.append(c)
    return np.asarray(pts, dtype=np.float64)


def signed_polygon_area(poly_uv: np.ndarray) -> float:
    # Shoelace formula; polygon is open trajectory -> close implicitly end->start.
    x = poly_uv[:, 0]
    y = poly_uv[:, 1]
    x2 = np.roll(x, -1)
    y2 = np.roll(y, -1)
    return 0.5 * float(np.sum(x * y2 - x2 * y))


def fmt_vec(v: np.ndarray) -> str:
    return f"[{v[0]: .6f}, {v[1]: .6f}, {v[2]: .6f}]"


def analyze(points_xyz: np.ndarray, times: np.ndarray) -> None:
    n = points_xyz.shape[0]
    if n < 3:
        raise ValueError("Need at least 3 pose samples.")

    # Basic step / path metrics
    diffs = points_xyz[1:] - points_xyz[:-1]
    step = np.linalg.norm(diffs, axis=1)
    path_len = float(np.sum(step))

    start = points_xyz[0]
    end = points_xyz[-1]
    closure_3d = float(np.linalg.norm(end - start))

    # PCA plane fit on translation trajectory
    center = np.mean(points_xyz, axis=0)
    centered = points_xyz - center
    # centered = U * S * Vt ; rows of Vt are principal directions
    _, s, vt = np.linalg.svd(centered, full_matrices=False)

    e1 = vt[0]  # strongest in-plane axis
    e2 = vt[1]  # second in-plane axis
    normal = vt[2]  # least-variance axis

    # Project onto best-fit plane basis
    u = centered @ e1
    v = centered @ e2
    uv = np.column_stack([u, v])

    # Distance-to-plane stats (signed via normal, absolute for RMS)
    d = centered @ normal
    rms_plane = float(np.sqrt(np.mean(d ** 2)))
    max_plane = float(np.max(np.abs(d)))

    # In-plane closure
    closure_uv = float(np.linalg.norm(uv[-1] - uv[0]))

    # Variance explained by plane
    var = s ** 2
    var_total = float(np.sum(var)) if np.sum(var) > 0 else 1.0
    var_plane = float(var[0] + var[1])
    var_plane_ratio = var_plane / var_total

    # Orientation / winding in fitted plane
    area = signed_polygon_area(uv)
    winding = "CCW" if area > 0 else ("CW" if area < 0 else "degenerate")

    duration = float(times[-1] - times[0]) if n > 1 else 0.0

    print("=== Trajectory Orientation / Planarity Report ===")
    print(f"samples: {n}")
    print(f"duration_s: {duration:.6f}")
    print(f"path_length_m: {path_len:.6f}")
    print(f"step_median_m: {float(np.median(step)):.6f}")
    print(f"step_max_m: {float(np.max(step)):.6f}")
    print()

    print("-- Closure --")
    print(f"closure_3d_m: {closure_3d:.6f}")
    print(f"closure_in_plane_m: {closure_uv:.6f}")
    if path_len > 1e-12:
        print(f"closure_3d_over_path: {closure_3d / path_len:.6f}")
        print(f"closure_plane_over_path: {closure_uv / path_len:.6f}")
    print()

    print("-- Best-fit plane --")
    print(f"centroid_xyz: {fmt_vec(center)}")
    print(f"plane_axis_e1_xyz: {fmt_vec(e1)}")
    print(f"plane_axis_e2_xyz: {fmt_vec(e2)}")
    print(f"plane_normal_xyz: {fmt_vec(normal)}")
    print(f"plane_rms_distance_m: {rms_plane:.6f}")
    print(f"plane_max_distance_m: {max_plane:.6f}")
    print(f"plane_variance_ratio: {var_plane_ratio:.6f}")
    print()

    print("-- In-plane orientation --")
    print(f"signed_area_plane_m2: {area:.6f}")
    print(f"winding_in_fitted_plane: {winding}")


def main() -> None:
    p = argparse.ArgumentParser(description="Analyze pose trajectory orientation/planarity/closure.")
    p.add_argument("--poses", required=True, type=Path, help="Path to ARposes*.txt")
    p.add_argument(
        "--axis-map",
        default="x,y,z",
        help="Output axis mapping from input coords, e.g. x,y,z (identity), x,z,-y, -x,y,z",
    )
    p.add_argument(
        "--pose-semantics",
        choices=["camera_center", "world_to_camera_t"],
        default="camera_center",
        help=(
            "Interpretation of tx,ty,tz in the pose file: "
            "camera_center=translation is camera position in world; "
            "world_to_camera_t=translation is extrinsic world->camera t (camera center computed as -R^T t)."
        ),
    )
    args = p.parse_args()

    rows = load_poses(args.poses)
    if not rows:
        raise SystemExit(f"No pose rows parsed from: {args.poses}")

    axis_map = parse_axis_map(args.axis_map)

    points = extract_points(rows, args.pose_semantics)
    points = remap_points(points, axis_map)
    times = np.array([r.t for r in rows], dtype=np.float64)

    print(f"input_file: {args.poses}")
    print(f"axis_map: {args.axis_map}")
    print(f"pose_semantics: {args.pose_semantics}")
    analyze(points, times)


if __name__ == "__main__":
    main()
