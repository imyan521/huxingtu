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

    @Test
    fun acceptsContinuousFullRevolution() {
        val angles = FloatArray(360) { it.toFloat() }

        val coverage = LidarScanFilter.inspectAngularCoverage(angles)

        assertEquals(true, coverage.isComplete)
        assertEquals(359f, coverage.coveredDegrees, 0.001f)
        assertEquals(1f, coverage.maximumGapDegrees, 0.001f)
    }

    @Test
    fun rejectsRevolutionWithMissingPacketSector() {
        val angles = (0..120).map(Int::toFloat) +
            (141..359).map(Int::toFloat)

        val coverage = LidarScanFilter.inspectAngularCoverage(angles.toFloatArray())

        assertEquals(false, coverage.isComplete)
        assertEquals(21f, coverage.maximumGapDegrees, 0.001f)
    }

    @Test
    fun rejectsShortUnsupportedFarReflection() {
        val ranges = FloatArray(360) { 2.0f }
        ranges[100] = 7.0f
        ranges[101] = 7.0f
        val result = LidarScanFilter.rejectIsolatedSpikes(
            ranges = ranges,
            anglesDeg = FloatArray(360) { it.toFloat() }
        )

        assertEquals(2, result.rejectedPointCount)
        assertEquals(358, result.ranges.size)
    }

    @Test
    fun retainsThreeRayFarWall() {
        val ranges = FloatArray(360) { 2.0f }
        ranges[100] = 7.0f
        ranges[101] = 7.0f
        ranges[102] = 7.0f
        val result = LidarScanFilter.rejectIsolatedSpikes(
            ranges = ranges,
            anglesDeg = FloatArray(360) { it.toFloat() }
        )

        assertEquals(0, result.rejectedPointCount)
    }
}
