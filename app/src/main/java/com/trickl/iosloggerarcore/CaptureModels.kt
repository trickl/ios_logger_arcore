package com.trickl.iosloggerarcore

import android.util.Size

data class ResolutionOption(val label: String, val size: Size)

data class CameraIntrinsics(
    val fx: Double,
    val fy: Double,
    val cx: Double,
    val cy: Double
)

data class PoseSample(
    val timestampSeconds: Double,
    val tx: Double,
    val ty: Double,
    val tz: Double,
    val qw: Double,
    val qx: Double,
    val qy: Double,
    val qz: Double,
)
