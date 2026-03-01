# Migration Notes (v1)

## If you previously consumed `ARposes.txt` as `qx qy qz qw`

Update parser logic: raw `ARposes.txt` uses `qw qx qy qz`.

## If you assumed constant intrinsics

`Frames.txt` can vary `fx,fy,cx,cy` per frame. Preserve per-frame values.

## If you linked poses by row index only

Use timestamp-based synchronization against `Frames.txt`; row counts/rates can differ.
