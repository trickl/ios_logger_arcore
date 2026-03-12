package com.trickl.iosloggerarcore

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs
import kotlin.math.sqrt

class ArKitConventionMapperTest {

    @Test
    fun mapPose_preservesTranslationAndTimestamp() {
        val out = ArKitConventionMapper.mapPose(
            timestampSeconds = 123.0,
            tx = 1.25,
            ty = -0.5,
            tz = 2.0,
            qx = 0.0,
            qy = 0.0,
            qz = 0.0,
            qw = 1.0,
        )

        assertEquals(123.0, out.timestampSeconds, 1e-9)
        assertEquals(-1.25, out.tx, 1e-9)
        assertEquals(-0.5, out.ty, 1e-9)
        assertEquals(2.0, out.tz, 1e-9)
    }

    @Test
    fun mapPose_appliesBaselineCameraCorrection() {
        val out = ArKitConventionMapper.mapPose(
            timestampSeconds = 0.0,
            tx = 0.0,
            ty = 0.0,
            tz = 0.0,
            qx = 0.0,
            qy = 0.0,
            qz = 0.0,
            qw = 1.0,
        )

        // Baseline mapper result should be a non-identity normalized rotation.
        val norm = sqrt(out.qx * out.qx + out.qy * out.qy + out.qz * out.qz + out.qw * out.qw)
        assertEquals(1.0, norm, 1e-9)
        assertTrue(abs(out.qw) < 0.999)
        assertTrue(abs(out.qx) + abs(out.qy) + abs(out.qz) > 1e-6)
    }

    @Test
    fun mapPose_normalizesQuaternion() {
        val out = ArKitConventionMapper.mapPose(
            timestampSeconds = 1.0,
            tx = 0.0,
            ty = 0.0,
            tz = 0.0,
            qx = 2.0,
            qy = 0.0,
            qz = 0.0,
            qw = 0.0,
        )

        val norm = sqrt(out.qx * out.qx + out.qy * out.qy + out.qz * out.qz + out.qw * out.qw)
        assertEquals(1.0, norm, 1e-9)
        assertFalse(out.qx.isNaN())
        assertFalse(out.qy.isNaN())
        assertFalse(out.qz.isNaN())
        assertFalse(out.qw.isNaN())
        assertTrue(norm > 0.999)
    }

    @Test
    fun mapPose_identityRemap_isOrientationIndependentAndAxisConsistent() {
        val out = ArKitConventionMapper.mapPose(
            timestampSeconds = 0.0,
            tx = 0.0,
            ty = 0.0,
            tz = 0.0,
            qx = 0.0,
            qy = 0.0,
            qz = 0.0,
            qw = 1.0,
        )

        val r = quaternionToRotation3x3(qx = out.qx, qy = out.qy, qz = out.qz, qw = out.qw)
        // Columns are camera axes in world coordinates.
        val cameraXinWorld = doubleArrayOf(r[0][0], r[1][0], r[2][0])
        val cameraYinWorld = doubleArrayOf(r[0][1], r[1][1], r[2][1])
        val cameraZinWorld = doubleArrayOf(r[0][2], r[1][2], r[2][2])

        // With baseline Rz(+90) camera correction composed with fixed export remap,
        // expected camera basis in world is approximately:
        // X -> -Y, Y -> +X, Z -> +Z.
        assertTrue(cameraXinWorld[1] < -0.5)
        assertTrue(cameraYinWorld[0] > 0.5)
        assertTrue(cameraZinWorld[2] > 0.5)
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

}
