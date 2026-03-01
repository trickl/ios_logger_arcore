package com.trickl.iosloggerarcore

import kotlin.math.sqrt

/**
 * Explicit pose-convention bridge from ARCore camera pose to the ARKit-style
 * output contract used by ios_logger (`ARposes.txt`):
 *
 *   time_s, tx, ty, tz, qw, qx, qy, qz
 *
 * Semantics target:
 * - camera-to-world transform (T_wc), not world-to-camera (T_cw)
 * - right-handed world frame, gravity-aligned Y-up
 * - quaternion scalar-first in output file
 *
 * IMPORTANT:
 * ARCore and ARKit camera/world conventions are very close for this use-case.
 * The basis-change matrix below is intentionally identity unless we have
 * verified evidence that a fixed axis remap is required.
 */
object ArKitConventionMapper {
    /**
     * Basis transform from ARCore coordinates to ARKit coordinates.
     *
     * If future audits prove a fixed axis remap is required, update this one
     * matrix and all pose outputs will follow automatically.
     */
    private val arCoreToArKitBasis: Array<DoubleArray> = arrayOf(
        doubleArrayOf(1.0, 0.0, 0.0),
        doubleArrayOf(0.0, 1.0, 0.0),
        doubleArrayOf(0.0, 0.0, 1.0),
    )

    private val arCoreToArKitBasisT: Array<DoubleArray> = transpose3x3(arCoreToArKitBasis)

    /**
     * Fixed camera-frame correction from ARCore camera axes to the dataset's
     * ARKit-compatible camera axes expected by downstream processing.
     *
     * Derived from three controlled captures:
     * 1) lateral sweep (expected local x dominance)
     * 2) vertical sweep (expected local y dominance)
     * 3) forward sweep (expected local -z / forward dominance)
     *
     * Without this correction, x/y are effectively swapped in local-motion
     * diagnostics. A -90° roll around camera Z aligns all three tests.
     */
    private val arCoreCameraToArKitCamera: Array<DoubleArray> = rotationZ3x3(degrees = -90.0)
    private val arCoreCameraToArKitCameraT: Array<DoubleArray> = transpose3x3(arCoreCameraToArKitCamera)

    fun mapPose(
        timestampSeconds: Double,
        tx: Double,
        ty: Double,
        tz: Double,
        qx: Double,
        qy: Double,
        qz: Double,
        qw: Double,
    ): PoseSample {
        val normalized = normalizeQuaternion(qx, qy, qz, qw)
        val rWcArCore = quaternionToRotation3x3(
            qx = normalized[0],
            qy = normalized[1],
            qz = normalized[2],
            qw = normalized[3],
        )

        // Change basis for both world and camera coordinates:
        // R_arkit = B * R_arcore * B^T,  t_arkit = B * t_arcore
        val rWcArKitBasis = multiply3x3(multiply3x3(arCoreToArKitBasis, rWcArCore), arCoreToArKitBasisT)
        // Camera-frame convention correction: R_new = R_old * C^T
        val rWcArKit = multiply3x3(rWcArKitBasis, arCoreCameraToArKitCameraT)
        val tArKit = multiply3x1(arCoreToArKitBasis, doubleArrayOf(tx, ty, tz))

        val mappedQuat = rotation3x3ToQuaternion(rWcArKit)

        return PoseSample(
            timestampSeconds = timestampSeconds,
            tx = tArKit[0],
            ty = tArKit[1],
            tz = tArKit[2],
            qw = mappedQuat[3],
            qx = mappedQuat[0],
            qy = mappedQuat[1],
            qz = mappedQuat[2],
        )
    }

    private fun normalizeQuaternion(qx: Double, qy: Double, qz: Double, qw: Double): DoubleArray {
        val n = sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if (n <= 1e-12) {
            return doubleArrayOf(0.0, 0.0, 0.0, 1.0)
        }
        return doubleArrayOf(qx / n, qy / n, qz / n, qw / n)
    }

    private fun quaternionToRotation3x3(qx: Double, qy: Double, qz: Double, qw: Double): Array<DoubleArray> {
        val xx = qx * qx
        val yy = qy * qy
        val zz = qz * qz
        val xy = qx * qy
        val xz = qx * qz
        val yz = qy * qz
        val wx = qw * qx
        val wy = qw * qy
        val wz = qw * qz

        return arrayOf(
            doubleArrayOf(1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
            doubleArrayOf(2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
            doubleArrayOf(2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
        )
    }

    private fun rotation3x3ToQuaternion(r: Array<DoubleArray>): DoubleArray {
        val trace = r[0][0] + r[1][1] + r[2][2]

        val qx: Double
        val qy: Double
        val qz: Double
        val qw: Double

        if (trace > 0.0) {
            val s = sqrt(trace + 1.0) * 2.0
            qw = 0.25 * s
            qx = (r[2][1] - r[1][2]) / s
            qy = (r[0][2] - r[2][0]) / s
            qz = (r[1][0] - r[0][1]) / s
        } else if (r[0][0] > r[1][1] && r[0][0] > r[2][2]) {
            val s = sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2.0
            qw = (r[2][1] - r[1][2]) / s
            qx = 0.25 * s
            qy = (r[0][1] + r[1][0]) / s
            qz = (r[0][2] + r[2][0]) / s
        } else if (r[1][1] > r[2][2]) {
            val s = sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2.0
            qw = (r[0][2] - r[2][0]) / s
            qx = (r[0][1] + r[1][0]) / s
            qy = 0.25 * s
            qz = (r[1][2] + r[2][1]) / s
        } else {
            val s = sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2.0
            qw = (r[1][0] - r[0][1]) / s
            qx = (r[0][2] + r[2][0]) / s
            qy = (r[1][2] + r[2][1]) / s
            qz = 0.25 * s
        }

        return normalizeQuaternion(qx, qy, qz, qw)
    }

    private fun multiply3x1(a: Array<DoubleArray>, b: DoubleArray): DoubleArray {
        return doubleArrayOf(
            a[0][0] * b[0] + a[0][1] * b[1] + a[0][2] * b[2],
            a[1][0] * b[0] + a[1][1] * b[1] + a[1][2] * b[2],
            a[2][0] * b[0] + a[2][1] * b[1] + a[2][2] * b[2],
        )
    }

    private fun multiply3x3(a: Array<DoubleArray>, b: Array<DoubleArray>): Array<DoubleArray> {
        val out = Array(3) { DoubleArray(3) }
        for (i in 0..2) {
            for (j in 0..2) {
                out[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j]
            }
        }
        return out
    }

    private fun transpose3x3(a: Array<DoubleArray>): Array<DoubleArray> {
        return arrayOf(
            doubleArrayOf(a[0][0], a[1][0], a[2][0]),
            doubleArrayOf(a[0][1], a[1][1], a[2][1]),
            doubleArrayOf(a[0][2], a[1][2], a[2][2]),
        )
    }

    private fun rotationZ3x3(degrees: Double): Array<DoubleArray> {
        val radians = Math.toRadians(degrees)
        val c = kotlin.math.cos(radians)
        val s = kotlin.math.sin(radians)
        return arrayOf(
            doubleArrayOf(c, -s, 0.0),
            doubleArrayOf(s, c, 0.0),
            doubleArrayOf(0.0, 0.0, 1.0),
        )
    }
}
