# ios_logger_arcore

Android logger inspired by `Varvrar/ios_logger` with a camera-first UI (`EasyCam`):

1. Fit-style preview (no crop-to-fill), with black letterbox margins when needed
2. Large centered record control (morphs between idle and recording states)
3. Settings button at the bottom-left of the control area
4. Recording status dot + tracking state indicator

Notes:

- There is no torch toggle in-app.
- Resolution is selected automatically from the full capture pipeline (highest supported back-camera recorder size).

## What this app currently records

Each recording session creates a timestamped folder under app-private external storage:

- `Frames.m4v` – recorded camera video
- `Frames.txt` – `time_s,frame_index,fx,fy,cx,cy` (or `time_s,frame_index` if intrinsics unavailable)
- `ARposes.txt` – `time_s,tx,ty,tz,qw,qx,qy,qz`
- `Accel.txt`
- `Gyro.txt`
- `GPS.txt`
- `Head.txt`
- `Motion.txt`
- `MotARH.txt`
- `MotMagnFull.txt`
- `Magnet.txt`
- `CaptureEvents.txt` – capture lifecycle/debug events (`timestamp,event,details`)

## ARCore mode (strict, no fallback)

- Pose source is strict ARCore camera pose only.
- If ARCore tracking is unavailable/lost, recording stops instead of writing zero/placeholder poses.
- Capture pipeline uses ARCore Shared Camera + Camera2 to avoid ARCore/CameraX camera ownership conflicts.
- Preview is active while the app is open (idle preview before recording + preview during recording).
- Pose continuity mitigation is applied on relocalization/reset-like jumps via world re-anchoring.

### Important current limitation

This app requires an ARCore-capable device and active tracking for capture; there is no non-AR fallback pose path.

## Open in Android Studio

1. Open `/home/timg/projects/ios_logger_arcore` as project.
2. Let Android Studio sync Gradle and generate wrapper files if prompted.
3. Run on:
	- a physical Android phone (recommended for camera), or
	- an emulator with camera support enabled.

## Run tests

- Unit tests: `TimeUtilsTest`, `DatasetWriterTest`
- Instrumentation UI test: `MainActivityTest`

From Android Studio:

- Run "All Tests" for local unit tests.
- Run "connectedAndroidTest" for instrumentation tests with emulator/device connected.

## Included spec docs

- `spec/ios_logger_dataset_spec.md`
- `spec/compatibility_profile_neuralrecon_demo.md`
- `spec/validation_checklist.md`
- `spec/migration_notes_v1.md`

## Pull + sync latest dataset (one command)

Use the helper script to copy the newest recording from device and run quick validation:

- `scripts/pull_latest_dataset.sh`

If multiple ADB devices are connected, the script automatically picks the first one reported by `adb devices` (and prints which one it chose).

Optional sync step (uses `../ios_logger/sync-data.py` by default):

- `scripts/pull_latest_dataset.sh --sync`

Create a labeled copy for experiments (saved under `pulled_datasets/labeled/`):

- `scripts/pull_latest_dataset.sh --label pure_yaw_clockwise`

Useful options:

- `--serial <adb_id>` to override auto-selected device
- `--dataset <folder_name>` to pull a specific capture
- `--label <name>` to create a labeled copy (`<dataset>__<name>`)
- `--dest-root <path>` to choose local destination
- `--no-validate` to skip summary checks
- `--sync-script <path>` to override sync script location
