# Validation Checklist (Pre-Converter Gate)

Use this checklist before marking a converter implementation as stable.

## Schema checks

- [ ] `Frames.txt` parsed with either 2 or 6 columns
- [ ] `ARposes.txt` parsed with exactly 8 columns
- [ ] Sensor files parse according to expected column counts
- [ ] All timestamps are finite and non-negative

## Ordering and integrity

- [ ] Streams are sorted by timestamp before synchronization
- [ ] Duplicate timestamps are handled deterministically
- [ ] Frame indices are monotonic and unique
- [ ] Missing frame IDs are either filled or explicitly reported

## Pose quality

- [ ] Quaternion reorder rules are explicit per output profile
- [ ] Quaternion normalization is enforced after transforms
- [ ] Pose transform direction is explicit and validated (`T_wc` camera-to-world for ARKit compatibility)
- [ ] Max frame-to-pose time gap is measured and reported
- [ ] Outlier jumps are flagged (translation and rotation deltas)

## Intrinsics quality

- [ ] `fx > 0`, `fy > 0`
- [ ] `cx,cy` are finite
- [ ] Per-frame intrinsics changes are preserved (no accidental constant override)

## Output package checks

- [ ] `images/`, `intrinsics/`, `poses/` filenames align with 5-digit frame IDs
- [ ] Row counts are consistent across frame-linked outputs
- [ ] Export contains no NaN/Inf values
- [ ] A metadata/report file records thresholds, drops, and interpolation choices

## Regression checks

- [ ] Same input produces byte-stable outputs where expected
- [ ] Small synthetic dataset test passes (known expected files)
- [ ] Real demo dataset conversion passes all checks above
