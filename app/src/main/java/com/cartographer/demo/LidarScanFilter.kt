package com.cartographer.demo

import kotlin.math.cos
import kotlin.math.sqrt

/**
 * Conservative geometry filter for one angle-ordered lidar revolution.
 *
 * A point is removed only when both adjacent rays disagree with it while the
 * two adjacent returns agree with each other. Real depth discontinuities at a
 * wall corner or doorway are therefore retained: on a real edge the two
 * neighbors normally do not describe the same surface.
 */
object LidarScanFilter {
    data class Result(
        val ranges: FloatArray,
        val anglesDeg: FloatArray,
        val rejectedPointCount: Int
    )

    fun rejectIsolatedSpikes(
        ranges: FloatArray,
        anglesDeg: FloatArray
    ): Result {
        val count = minOf(ranges.size, anglesDeg.size)
        if (count < 3) {
            return Result(
                ranges.copyOf(count),
                anglesDeg.copyOf(count),
                rejectedPointCount = 0
            )
        }

        val keep = BooleanArray(count) { true }
        // A complete revolution is circular. Treat its first/last samples as
        // neighbors too, otherwise a corrupt return at the packet boundary is
        // the only isolated spike that can never be rejected. For a partial
        // scan the wrapped angle gap is large and the existing gap guard skips
        // the boundary naturally.
        for (index in 0 until count) {
            val previousIndex = (index + count - 1) % count
            val nextIndex = (index + 1) % count
            val previousAngleGap = forwardAngleDelta(
                anglesDeg[previousIndex],
                anglesDeg[index]
            )
            val nextAngleGap = forwardAngleDelta(
                anglesDeg[index],
                anglesDeg[nextIndex]
            )
            if (previousAngleGap !in MIN_ANGLE_GAP_DEG..MAX_ANGLE_GAP_DEG ||
                nextAngleGap !in MIN_ANGLE_GAP_DEG..MAX_ANGLE_GAP_DEG) {
                continue
            }

            val previousRange = ranges[previousIndex]
            val currentRange = ranges[index]
            val nextRange = ranges[nextIndex]
            if (!previousRange.isFinite() || !currentRange.isFinite() ||
                !nextRange.isFinite() || previousRange <= 0f ||
                currentRange <= 0f || nextRange <= 0f) {
                continue
            }

            val previousDistance = polarPointDistance(
                previousRange,
                currentRange,
                previousAngleGap
            )
            val nextDistance = polarPointDistance(
                currentRange,
                nextRange,
                nextAngleGap
            )
            val neighborDistance = polarPointDistance(
                previousRange,
                nextRange,
                previousAngleGap + nextAngleGap
            )

            // Angular sample spacing grows naturally with range. Scale the
            // thresholds with that spacing so valid far walls are not treated
            // as sparse outliers. These deliberately loose limits target only
            // one-ray spikes, not small furniture or wall corners.
            val currentGapRadians = Math.toRadians(
                maxOf(previousAngleGap, nextAngleGap).toDouble()
            ).toFloat()
            val currentThreshold = BASE_POINT_GAP_M +
                currentRange * currentGapRadians * CURRENT_GAP_MULTIPLIER
            val neighborGapRadians = Math.toRadians(
                (previousAngleGap + nextAngleGap).toDouble()
            ).toFloat()
            val neighborThreshold = BASE_NEIGHBOR_SUPPORT_M +
                minOf(previousRange, nextRange) * neighborGapRadians *
                NEIGHBOR_SUPPORT_MULTIPLIER

            if (previousDistance > currentThreshold &&
                nextDistance > currentThreshold &&
                neighborDistance <= neighborThreshold) {
                keep[index] = false
            }
        }

        // Long-range multipath/dark-surface echoes often arrive as a short
        // cluster rather than one bad ray. Reject a far return when neither
        // side of its small angular neighborhood supports the same surface.
        // The comparison is radial (not Euclidean), so a real far wall with
        // several consecutive samples survives while 1-3 ray "tails" do not.
        for (index in 0 until count) {
            if (!keep[index] || ranges[index] < LONG_RANGE_CHECK_START_M) continue
            var supportedBefore = false
            var supportedAfter = false
            for (offset in 1..LONG_RANGE_SUPPORT_RADIUS) {
                val before = (index + count - offset) % count
                val after = (index + offset) % count
                val beforeGap = forwardAngleDelta(anglesDeg[before], anglesDeg[index])
                val afterGap = forwardAngleDelta(anglesDeg[index], anglesDeg[after])
                if (beforeGap <= LONG_RANGE_MAX_SUPPORT_ANGLE_DEG &&
                    keep[before] &&
                    kotlin.math.abs(ranges[before] - ranges[index]) <= LONG_RANGE_RADIAL_TOLERANCE_M) {
                    supportedBefore = true
                }
                if (afterGap <= LONG_RANGE_MAX_SUPPORT_ANGLE_DEG &&
                    keep[after] &&
                    kotlin.math.abs(ranges[after] - ranges[index]) <= LONG_RANGE_RADIAL_TOLERANCE_M) {
                    supportedAfter = true
                }
            }
            if (!supportedBefore && !supportedAfter) keep[index] = false
        }

        val rejected = keep.count { !it }

        if (rejected == 0) {
            return Result(
                ranges.copyOf(count),
                anglesDeg.copyOf(count),
                rejectedPointCount = 0
            )
        }

        val filteredRanges = FloatArray(count - rejected)
        val filteredAngles = FloatArray(count - rejected)
        var outputIndex = 0
        for (index in 0 until count) {
            if (!keep[index]) continue
            filteredRanges[outputIndex] = ranges[index]
            filteredAngles[outputIndex] = anglesDeg[index]
            outputIndex++
        }
        return Result(filteredRanges, filteredAngles, rejected)
    }

    private fun forwardAngleDelta(fromDeg: Float, toDeg: Float): Float {
        var delta = (toDeg - fromDeg) % 360f
        if (delta < 0f) delta += 360f
        return delta
    }

    private fun polarPointDistance(
        firstRange: Float,
        secondRange: Float,
        angleDeltaDeg: Float
    ): Float {
        val deltaRadians = Math.toRadians(angleDeltaDeg.toDouble())
        val squared = firstRange * firstRange + secondRange * secondRange -
            2f * firstRange * secondRange * cos(deltaRadians).toFloat()
        return sqrt(squared.coerceAtLeast(0f))
    }

    private const val MIN_ANGLE_GAP_DEG = 0.01f
    private const val MAX_ANGLE_GAP_DEG = 2.5f
    private const val BASE_POINT_GAP_M = 0.08f
    private const val CURRENT_GAP_MULTIPLIER = 3.0f
    private const val BASE_NEIGHBOR_SUPPORT_M = 0.10f
    private const val NEIGHBOR_SUPPORT_MULTIPLIER = 2.0f
    private const val LONG_RANGE_CHECK_START_M = 5.0f
    private const val LONG_RANGE_SUPPORT_RADIUS = 3
    private const val LONG_RANGE_MAX_SUPPORT_ANGLE_DEG = 2.5f
    private const val LONG_RANGE_RADIAL_TOLERANCE_M = 0.35f
}
