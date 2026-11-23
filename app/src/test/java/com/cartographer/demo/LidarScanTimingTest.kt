package com.cartographer.demo

import org.junit.Assert.*
import org.junit.Test

class LidarScanTimingTest {
    @Test fun phaseCorrectionCannotOverlapThePreviousRevolution() {
        val previous = 10_000_000_000L
        for (interval in listOf(95_000_000L, 99_500_000L, 100_000_000L, 105_000_000L)) {
            val end = previous + interval
            val duration = LidarScanTiming.durationSeconds(100_000_000L, end, previous)!!
            assertTrue(end - duration.toDouble() * 1e9 > previous)
        }
    }
    @Test fun realPauseDoesNotStretchOneScan() {
        val duration = LidarScanTiming.durationSeconds(100_000_000L, 12_000_000_000L, 10_000_000_000L)!!
        assertEquals(0.1, duration.toDouble(), 1e-7)
    }
    @Test fun impossibleShortOrBackwardIntervalsAreRejected() {
        assertNull(LidarScanTiming.durationSeconds(100_000_000L, 1_020_000_000L, 1_000_000_000L))
        assertNull(LidarScanTiming.durationSeconds(100_000_000L, 999_000_000L, 1_000_000_000L))
    }
    @Test fun firstScanUsesTheMeasuredPeriod() {
        assertEquals(.1, LidarScanTiming.durationSeconds(100_000_000L, 1_000_000_000L, 0L)!!.toDouble(), 1e-7)
    }
}
