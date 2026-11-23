package com.cartographer.demo

import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Test

class LidarScanFilterTest {
    @Test
    fun removesSingleRaySpikeBetweenConsistentNeighbors() {
        val result = LidarScanFilter.rejectIsolatedSpikes(
            ranges = floatArrayOf(2.0f, 7.0f, 2.0f),
            anglesDeg = floatArrayOf(10.0f, 10.5f, 11.0f)
        )

        assertEquals(1, result.rejectedPointCount)
        assertArrayEquals(floatArrayOf(2.0f, 2.0f), result.ranges, 0.0001f)
        assertArrayEquals(floatArrayOf(10.0f, 11.0f), result.anglesDeg, 0.0001f)
    }

    @Test
    fun retainsRealDepthDiscontinuity() {
        val result = LidarScanFilter.rejectIsolatedSpikes(
            ranges = floatArrayOf(2.0f, 2.0f, 6.0f),
            anglesDeg = floatArrayOf(10.0f, 10.5f, 11.0f)
        )

        assertEquals(0, result.rejectedPointCount)
        assertArrayEquals(floatArrayOf(2.0f, 2.0f, 6.0f), result.ranges, 0.0001f)
    }

    @Test
    fun retainsDistantDoorEdgeBesideMissingSector() {
        val ranges = floatArrayOf(2f, 2f, 2f, 6f)
        val result = LidarScanFilter.rejectIsolatedSpikes(
            ranges, floatArrayOf(10f, 10.5f, 11f, 11.5f))
        assertArrayEquals(ranges, result.ranges, 0.0001f)
    }

    @Test
    fun retainsGrazingWallEvenWhenNeighborRangesDiffer() {
        // Samples all lie on the same straight wall y=1 m. Radial distance
        // changes rapidly at grazing incidence; this is not isolated noise.
        val angles = floatArrayOf(8f, 9f, 10f, 11f, 12f)
        val ranges = FloatArray(angles.size) {
            (1.0 / kotlin.math.sin(Math.toRadians(angles[it].toDouble()))).toFloat()
        }
        val result = LidarScanFilter.rejectIsolatedSpikes(ranges, angles)
        assertArrayEquals(ranges, result.ranges, 0.0001f)
    }

    @Test
    fun stillRejectsUnsupportedFarTailBetweenConsistentNearReturns() {
        val result = LidarScanFilter.rejectIsolatedSpikes(
            floatArrayOf(2f, 2f, 6f, 7f, 2f, 2f),
            floatArrayOf(10f, 10.5f, 11f, 11.5f, 12f, 12.5f))
        assertEquals(2, result.rejectedPointCount)
        assertArrayEquals(floatArrayOf(2f, 2f, 2f, 2f), result.ranges, 0.0001f)
    }

    @Test
    fun changingTheRevolutionStartDoesNotChangeTheKeptPoints() {
        val ranges = floatArrayOf(2f, 2f, 6f, 7f, 2f, 2f)
        val angles = floatArrayOf(10f, 10.5f, 11f, 11.5f, 12f, 12.5f)
        val original = LidarScanFilter.rejectIsolatedSpikes(ranges, angles)
        for (shift in 1 until ranges.size) {
            val shifted = LidarScanFilter.rejectIsolatedSpikes(
                FloatArray(ranges.size) { ranges[(it + shift) % ranges.size] },
                FloatArray(angles.size) { angles[(it + shift) % angles.size] })
            assertEquals(original.rejectedPointCount, shifted.rejectedPointCount)
            assertEquals(original.anglesDeg.toSet(), shifted.anglesDeg.toSet())
        }
    }

    @Test
    fun retainsFarWallWithNormalAngularSpacing() {
        val result = LidarScanFilter.rejectIsolatedSpikes(
            ranges = floatArrayOf(11.8f, 12.0f, 11.8f),
            anglesDeg = floatArrayOf(40.0f, 41.0f, 42.0f)
        )

        assertEquals(0, result.rejectedPointCount)
    }

    @Test
    fun doesNotBridgeMissingAngularSector() {
        val result = LidarScanFilter.rejectIsolatedSpikes(
            ranges = floatArrayOf(2.0f, 8.0f, 2.0f),
            anglesDeg = floatArrayOf(10.0f, 20.0f, 21.0f)
        )

        assertEquals(0, result.rejectedPointCount)
    }

    @Test
    fun removesSpikeAtRevolutionBoundary() {
        val result = LidarScanFilter.rejectIsolatedSpikes(
            ranges = floatArrayOf(7.0f, 2.0f, 2.0f, 2.0f),
            anglesDeg = floatArrayOf(0.0f, 0.5f, 180.0f, 359.5f)
        )

        assertEquals(1, result.rejectedPointCount)
        assertArrayEquals(
            floatArrayOf(2.0f, 2.0f, 2.0f),
            result.ranges,
            0.0001f
        )
    }
}
