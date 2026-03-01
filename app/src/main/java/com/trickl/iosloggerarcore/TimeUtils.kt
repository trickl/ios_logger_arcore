package com.trickl.iosloggerarcore

import android.os.SystemClock

object TimeUtils {
    fun bootToUnixOffsetSeconds(nowUnixSeconds: Double = System.currentTimeMillis() / 1000.0): Double {
        val elapsed = SystemClock.elapsedRealtimeNanos() / 1_000_000_000.0
        return nowUnixSeconds - elapsed
    }

    fun toUnixSeconds(timestampNanosSinceBoot: Long, bootToUnixOffsetSeconds: Double): Double {
        return bootToUnixOffsetSeconds + (timestampNanosSinceBoot / 1_000_000_000.0)
    }
}
