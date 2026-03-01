# Compatibility Profile: NeuralRecon Demo Layout

Profile target: dataset layout like `neucon_demodata_b5f1`.

## 1) Observed target files

- `Frames.txt`
- `ARposes.txt`
- `SyncedPoses.txt`
- `images/` (frame images)
- `intrinsics/00000.txt ...`
- `poses/00000.txt ...`

## 2) Mapping rules

### 2.1 Frame index

Use `frame_index` from `Frames.txt` as canonical image/intrinsics/pose basename:

- image: `images/<frame:05d>.{png|jpg}`
- intrinsics: `intrinsics/<frame:05d>.txt`
- pose: `poses/<frame:05d>.txt`

### 2.2 Intrinsics

From each `Frames.txt` row:

- Build $K = \begin{bmatrix}fx & 0 & cx\\0 & fy & cy\\0 & 0 & 1\end{bmatrix}$
- Write to `intrinsics/<frame:05d>.txt` as 3x3

### 2.3 Pose source and quaternion order

Raw iOS AR pose row is:

`time, tx, ty, tz, qw, qx, qy, qz`

Observed `SyncedPoses.txt` row format is:

`<frame:05d> tx ty tz qx qy qz qw`

Therefore for this profile:

- Reorder quaternion from `(qw,qx,qy,qz)` to `(qx,qy,qz,qw)`
- Keep translation in meters
- Keep frame-linked pose rows synchronized to `Frames.txt` timeline

### 2.4 Alignment policy

For each frame timestamp:

- select nearest AR pose by timestamp
- optional: interpolate translation + slerp quaternion if both neighbors exist
- reject/flag if nearest gap exceeds threshold (default: 0.1 s)

## 3) Coordinate-frame caution

ARKit axis conventions and target consumer conventions may differ. This profile does **not** apply axis flips by default; apply explicit transform only after validating trajectory orientation with visual checks.

## 4) Minimal acceptance checks

- row count(`SyncedPoses`) == row count(`Frames`)
- frame ids are contiguous and match generated file names
- quaternion norm within tolerance (e.g. $|\|q\|-1| < 10^{-3}$)
- all intrinsics matrices have positive `fx`,`fy`
- no NaN/Inf in exported files
