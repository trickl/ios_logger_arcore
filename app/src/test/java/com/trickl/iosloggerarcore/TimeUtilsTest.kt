package com.trickl.iosloggerarcore

import org.junit.Assert.assertEquals
import org.junit.Test

class TimeUtilsTest {
    @Test
    fun `toUnixSeconds converts boot nanos deterministically`() {
        val offset = 1_700_000_000.0
        val nanos = 2_500_000_000L
        val unix = TimeUtils.toUnixSeconds(nanos, offset)
        assertEquals(1_700_000_002.5, unix, 1e-9)
    }
}
