# iOS Logger Dataset Spec (v1)

This document specifies file formats produced by `ios_logger` and clarifies field ordering, units, and known edge cases.

## 1) Core files

### 1.1 `Frames.txt`

CSV, no header:

`time_s,frame_index,fx,fy,cx,cy`

- `time_s`: Unix timestamp in seconds (`double`)
- `frame_index`: non-negative integer, starts from 0
- Intrinsics (pixel units):
  - `fx`: focal length x
  - `fy`: focal length y
  - `cx`: principal point x
  - `cy`: principal point y

Notes:
- In code, row is written as `%f,%lu,%f,%f,%f,%f`.
- If intrinsics are unavailable, only `time_s,frame_index` may appear.
- Android ARCore capture writes intrinsics in the recorded `Frames.m4v` pixel coordinate system. When ARCore reports intrinsics in a different native image size/aspect, values are remapped using centered crop + scale to match output resolution.

### 1.2 `ARposes.txt`

CSV, no header:

`time_s,tx,ty,tz,qw,qx,qy,qz`

- Translation in meters (`tx,ty,tz`), ARKit world frame
- Quaternion from camera transform (`qw,qx,qy,qz`)

Notes:
- In code, values are appended as `quat.vector[3], quat.vector[0], quat.vector[1], quat.vector[2]`, i.e. scalar-first output (`w,x,y,z`).
- Android ARCore capture normalizes pose timestamps to be strictly increasing. If ARCore returns a non-increasing timestamp, writer bumps it by 1 microsecond to preserve order for downstream sync code.
- Android ARCore capture explicitly exports **camera-to-world** pose (`T_wc`) to match ARKit `frame.camera.transform` semantics (no inversion to `T_cw`).
- ARCore pose components are routed through an explicit ARCore→ARKit convention mapper before writing.
- Mapper applies a fixed camera-frame roll correction of **-90° around camera Z** (in addition to explicit basis handling) to align pose camera axes with dataset/video conventions validated by controlled x/y/z motion captures.

### 1.3 `Frames.m4v`

Video stream of captured frames, appended in `frame_index` order.

## 2) Sensor files (optional toggles)

### 2.1 `Accel.txt`

`time_s,ax_g,ay_g,az_g` (g-units)

### 2.2 `Gyro.txt`

`time_s,gx_rad_s,gy_rad_s,gz_rad_s`

### 2.3 `Motion.txt`

`time_s,qw,qx,qy,qz` (CoreMotion attitude quaternion)

### 2.4 `MotARH.txt`

`time_s,rrx,rry,rrz,gx,gy,gz,uax,uay,uaz,heading_deg`

### 2.5 `MotMagnFull.txt`

`time_s,mx_uT,my_uT,mz_uT,accuracy_enum`

### 2.6 `Magnet.txt`

`time_s,mx_uT,my_uT,mz_uT`

### 2.7 `GPS.txt`

`time_s,lat_deg,lon_deg,hacc_m,alt_m,vacc_m,floor,course_deg,speed_m_s`

### 2.8 `Head.txt`

`time_s,true_heading_deg,mag_heading_deg,heading_acc_deg`

## 3) Time semantics

- Camera/sensor streams are timestamped with Unix time in seconds.
- Multiple streams are asynchronous and must be aligned by timestamp.

## 4) Known edge cases

- Repeated AR pose rows can occur (device static or tracking stalls).
- `Frames.txt` may include variable intrinsics between frames.
- `ARposes.txt` and sensor streams may have different rates and start/stop offsets.
- Minor duplicate timestamps can occur around writer/session transitions.

## 5) Normalization recommendations

For downstream conversion:

1. Sort each stream by `time_s` (stable sort).
2. Preserve original raw files unmodified.
3. When resampling to frame times, document interpolation mode and max gap.
4. Enforce quaternion normalization after reorder/transform operations.

## 6) Controlled reference captures

The following labeled capture is the current baseline reference for portrait-mode square-loop validation:

- **Portrait square reference (parallel wall):**
  - `pulled_datasets/labeled/2026-03-01T15-49-30__square_path_portrait_parallel_wall_retry`
  - Intended motion: square path while camera faces wall (portrait orientation).
  - Validation role: catch axis swapping and quantify loop-closure/drift coupling in the expected XY plane.

- **Landscape-right square reference (parallel wall):**
  - `pulled_datasets/labeled/2026-03-01T15-52-27__square_path_landscape_right_parallel_wall`
  - Intended motion: same square-path protocol as portrait reference, with device in landscape-right.
  - Validation role: confirm orientation-invariant XY-loop behavior and detect orientation-dependent axis swapping.
