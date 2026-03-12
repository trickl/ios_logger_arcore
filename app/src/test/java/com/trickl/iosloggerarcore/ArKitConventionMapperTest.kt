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

}
