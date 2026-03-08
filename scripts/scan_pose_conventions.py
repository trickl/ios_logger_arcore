#!/usr/bin/env python3
"""
Scan common pose convention variants and rank by geometry metrics.

Variants scanned:
- pose semantics:
    1) camera_center      (tx,ty,tz interpreted as camera position)
    2) world_to_camera_t  (camera center computed as C = -R^T t)
- axis maps:
    all 48 signed permutations of x,y,z (3! * 2^3)

Outputs top-N ranked variants with:
- closure ratio
- plane RMS / max distance
- plane variance ratio
- signed area + winding

Example:
  python scripts/scan_pose_conventions.py \
      --poses pulled_datasets/labeled/.../ARposes_legacy.txt \
      --top 15
"""

from __future__ import annotations

import argparse
import itertools
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


@dataclass
class VariantResult:
    semantics: str
    axis_map: str
    n: int
    path_length: float
    closure_3d: float
    closure_plane: float
    closure_ratio: float
    plane_rms: float
    plane_max: float
    plane_var_ratio: float
    signed_area: float
    winding: str
    score: float


def parse_line(line: str) -> List[float]:
    s = line.strip()
    if not s:
        return []
    parts = [p.strip() for p in s.split(",")] if "," in s else s.split()
    if len(parts) < 8:
        return []
    try:
        return [float(x) for x in parts[:8]]
    except ValueError:
        return []


def load_poses(path: Path) -> List[PoseRow]:
    rows: List[PoseRow] = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            vals = parse_line(line)
            if vals:
                rows.append(PoseRow(*vals))
    return rows


def quat_wxyz_to_rot(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
    n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if n <= 1e-12:
        qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0
    else:
        qw, qx, qy, qz = qw / n, qx / n, qy / n, qz / n

    return np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ],
        dtype=np.float64,
    )


def camera_points(rows: List[PoseRow], semantics: str) -> np.ndarray:
    pts = []
    for r in rows:
        t = np.array([r.tx, r.ty, r.tz], dtype=np.float64)
        if semantics == "camera_center":
            c = t
        elif semantics == "world_to_camera_t":
            rwc = quat_wxyz_to_rot(r.qw, r.qx, r.qy, r.qz)
            c = -(rwc.T @ t)
        else:
            raise ValueError(f"Unknown semantics: {semantics}")
        pts.append(c)
    return np.asarray(pts, dtype=np.float64)


def axis_map_to_str(m: Tuple[Tuple[int, float], Tuple[int, float], Tuple[int, float]]) -> str:
    names = ["x", "y", "z"]
    out = []
    for idx, sign in m:
        tok = names[idx]
        if sign < 0:
            tok = "-" + tok
        out.append(tok)
    return ",".join(out)


def all_axis_maps() -> List[Tuple[Tuple[int, float], Tuple[int, float], Tuple[int, float]]]:
    maps = []
    for perm in itertools.permutations([0, 1, 2], 3):
        for signs in itertools.product([-1.0, 1.0], repeat=3):
            maps.append(((perm[0], signs[0]), (perm[1], signs[1]), (perm[2], signs[2])))
    return maps


def apply_axis_map(points_xyz: np.ndarray, m: Tuple[Tuple[int, float], Tuple[int, float], Tuple[int, float]]) -> np.ndarray:
    out = np.zeros_like(points_xyz)
    for out_i, (in_i, sign) in enumerate(m):
        out[:, out_i] = sign * points_xyz[:, in_i]
    return out


def signed_area(poly_uv: np.ndarray) -> float:
    x = poly_uv[:, 0]
    y = poly_uv[:, 1]
    x2 = np.roll(x, -1)
    y2 = np.roll(y, -1)
    return 0.5 * float(np.sum(x * y2 - x2 * y))


def evaluate(points_xyz: np.ndarray) -> Tuple[float, float, float, float, float, float, float, str]:
    n = points_xyz.shape[0]
    if n < 3:
        raise ValueError("Need >= 3 points")

    diffs = points_xyz[1:] - points_xyz[:-1]
    step = np.linalg.norm(diffs, axis=1)
    path = float(np.sum(step))

    closure_3d = float(np.linalg.norm(points_xyz[-1] - points_xyz[0]))

    center = np.mean(points_xyz, axis=0)
    centered = points_xyz - center
    _, s, vt = np.linalg.svd(centered, full_matrices=False)
    e1, e2, nrm = vt[0], vt[1], vt[2]

    uv = np.column_stack([centered @ e1, centered @ e2])
    d = centered @ nrm

    closure_plane = float(np.linalg.norm(uv[-1] - uv[0]))
    rms = float(np.sqrt(np.mean(d ** 2)))
    maxd = float(np.max(np.abs(d)))

    var = s ** 2
    var_ratio = float((var[0] + var[1]) / max(np.sum(var), 1e-12))

    area = signed_area(uv)
    winding = "CCW" if area > 0 else ("CW" if area < 0 else "degenerate")

    closure_ratio = closure_3d / max(path, 1e-12)
    return path, closure_3d, closure_plane, closure_ratio, rms, maxd, var_ratio, winding, area


def main() -> None:
    ap = argparse.ArgumentParser(description="Scan pose conventions and rank orientation/planarity fits.")
    ap.add_argument("--poses", required=True, type=Path, help="Path to ARposes*.txt")
    ap.add_argument("--top", type=int, default=12, help="How many top variants to print")
    args = ap.parse_args()

    rows = load_poses(args.poses)
    if len(rows) < 3:
        raise SystemExit(f"Not enough rows parsed from {args.poses}")

    semantics_options = ["camera_center", "world_to_camera_t"]
    maps = all_axis_maps()

    results: List[VariantResult] = []

    for sem in semantics_options:
        base_pts = camera_points(rows, sem)
        for m in maps:
            pts = apply_axis_map(base_pts, m)
            path, c3, cp, cr, rms, maxd, varr, winding, area = evaluate(pts)

            # Lower is better: prefer planar + better closure.
            # Weighted to avoid trivially tiny trajectories (path term normalizes closure separately).
            score = (2.0 * cr) + (1.5 * rms) + (0.5 * maxd) + (1.0 - varr)

            results.append(
                VariantResult(
                    semantics=sem,
                    axis_map=axis_map_to_str(m),
                    n=len(rows),
                    path_length=path,
                    closure_3d=c3,
                    closure_plane=cp,
                    closure_ratio=cr,
                    plane_rms=rms,
                    plane_max=maxd,
                    plane_var_ratio=varr,
                    signed_area=area,
                    winding=winding,
                    score=score,
                )
            )

    results.sort(key=lambda r: r.score)

    print(f"input_file: {args.poses}")
    print(f"rows: {len(rows)}")
    print(f"variants_scanned: {len(results)}")
    print()
    print("rank,score,semantics,axis_map,closure_ratio,plane_rms_m,plane_max_m,plane_var_ratio,winding,signed_area_m2,path_m,closure3d_m,closure_plane_m")

    for i, r in enumerate(results[: max(1, args.top)], start=1):
        print(
            f"{i},{r.score:.6f},{r.semantics},{r.axis_map},"
            f"{r.closure_ratio:.6f},{r.plane_rms:.6f},{r.plane_max:.6f},{r.plane_var_ratio:.6f},"
            f"{r.winding},{r.signed_area:.6f},{r.path_length:.6f},{r.closure_3d:.6f},{r.closure_plane:.6f}"
        )


if __name__ == "__main__":
    main()
