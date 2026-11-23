package com.cartographer.demo

internal object LidarScanTiming {
    // Match the JNI limits. Leave a small margin for float conversion of
    // point-relative times so adjacent revolutions cannot overlap.
    fun durationSeconds(periodNs: Long, endNs: Long, previousEndNs: Long): Float? {
        val available = if (previousEndNs > 0L) endNs - previousEndNs - 1_000L
                        else Long.MAX_VALUE
        val duration = minOf(periodNs.coerceIn(40_000_000L, 400_000_000L), available)
        if (duration < 40_000_000L) return null
        return duration.toFloat() / 1_000_000_000f
    }
}
