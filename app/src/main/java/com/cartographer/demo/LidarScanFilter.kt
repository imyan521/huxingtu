package com.cartographer.demo

import kotlin.math.cos
import kotlin.math.sqrt

/**
 * Conservative geometry filter for one angle-ordered lidar revolution.
 *
 * Single-ray spikes are removed only when both adjacent rays disagree with the
 * point while agreeing with each other. On full revolutions, one/two-ray far
 * clusters additionally require local support so reflections cannot create a
 * long free-space wedge. Real depth discontinuities and sustained far walls
 * are retained.
 */
object LidarScanFilter {
    data class AngularCoverage(
        val sampleCount: Int,
        val coveredDegrees: Float,
        val maximumGapDegrees: Float,
        val isComplete: Boolean
    )

    data class Result(
        val ranges: FloatArray,
        val anglesDeg: FloatArray,
        val rejectedPointCount: Int
    )

    /**
     * Verifies that a scan assembled between two revolution boundaries really
     * contains one continuous turn. A zero-degree boundary alone is not enough:
     * a dropped USB packet can leave a large missing sector in the middle while
     * the next boundary still arrives normally.
     *
     * This check deliberately uses every decoded sample angle, including rays
     * without a usable range return. Open doorways and absorptive surfaces must
     * therefore not make an otherwise complete revolution fail validation.
     */
    fun inspectAngularCoverage(sampleAnglesDeg: FloatArray): AngularCoverage {
        if (sampleAnglesDeg.size < MIN_COMPLETE_SCAN_SAMPLES) {
            return AngularCoverage(
                sampleCount = sampleAnglesDeg.size,
                coveredDegrees = 0f,
                maximumGapDegrees = 360f,
                isComplete = false
            )
        }

        var coveredDegrees = 0f
        var maximumGapDegrees = 0f
        for (index in 1 until sampleAnglesDeg.size) {
            val gap = forwardAngleDelta(
                sampleAnglesDeg[index - 1],
                sampleAnglesDeg[index]
            )
            if (!gap.isFinite() || gap > MAX_FORWARD_SAMPLE_GAP_DEG) {
                maximumGapDegrees = maxOf(maximumGapDegrees, gap)
                continue
            }
            coveredDegrees += gap
            maximumGapDegrees = maxOf(maximumGapDegrees, gap)
        }
        val complete = coveredDegrees >= MIN_COMPLETE_SCAN_COVERAGE_DEG &&
            maximumGapDegrees <= MAX_COMPLETE_SCAN_GAP_DEG
        return AngularCoverage(
            sampleCount = sampleAnglesDeg.size,
            coveredDegrees = coveredDegrees,
            maximumGapDegrees = maximumGapDegrees,
            isComplete = complete
        )
    }

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
        var rejected = 0
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
                rejected++
            }
        }

        // A two-ray reflection is not removed by the single-spike rule above:
        // each bad ray sees the other bad ray as a neighbor. At long range that
        // tiny cluster produces a large free-space wedge and expands every
        // submap. Require two distinct nearby supporters for distant returns.
        // Three or more consecutive samples on a real far wall are retained.
        val unsupportedFarReturns = ArrayList<Int>()
        // Small arrays are also used by previews/tests and may describe only a
        // sector. Short-cluster rejection is valid only for a full-size scan.
        if (count >= MIN_COMPLETE_SCAN_SAMPLES) {
            for (index in 0 until count) {
                if (!keep[index] || ranges[index] < FAR_RETURN_START_M) continue
                val seenIndices = IntArray(FAR_SUPPORT_OFFSETS.size)
                var seenCount = 0
                var supportCount = 0
                for (offset in FAR_SUPPORT_OFFSETS) {
                    val neighborIndex = (index + offset + count) % count
                    if (neighborIndex == index || !keep[neighborIndex]) continue
                    var alreadySeen = false
                    for (seenIndex in 0 until seenCount) {
                        if (seenIndices[seenIndex] == neighborIndex) {
                            alreadySeen = true
                            break
                        }
                    }
                    if (alreadySeen) continue
                    seenIndices[seenCount++] = neighborIndex

                    val angleGap = circularAngleDistance(
                        anglesDeg[index],
                        anglesDeg[neighborIndex]
                    )
                    if (angleGap > FAR_SUPPORT_ANGLE_DEG) continue
                    val spatialDistance = polarPointDistance(
                        ranges[index],
                        ranges[neighborIndex],
                        angleGap
                    )
                    val angularSpacing = Math.toRadians(angleGap.toDouble()).toFloat()
                    val supportThreshold = FAR_SUPPORT_BASE_M +
                        minOf(ranges[index], ranges[neighborIndex]) *
                        angularSpacing * FAR_SUPPORT_SPACING_MULTIPLIER
                    if (spatialDistance <= supportThreshold) supportCount++
                }
                if (supportCount < MIN_FAR_SUPPORT_COUNT) {
                    unsupportedFarReturns += index
                }
            }
        }
        for (index in unsupportedFarReturns) {
            if (!keep[index]) continue
            keep[index] = false
            rejected++
        }

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

    private fun circularAngleDistance(firstDeg: Float, secondDeg: Float): Float {
        val forward = forwardAngleDelta(firstDeg, secondDeg)
        return minOf(forward, 360f - forward)
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
    private const val MIN_COMPLETE_SCAN_SAMPLES = 120
    private const val MIN_COMPLETE_SCAN_COVERAGE_DEG = 330f
    private const val MAX_COMPLETE_SCAN_GAP_DEG = 5f
    private const val MAX_FORWARD_SAMPLE_GAP_DEG = 45f
    private const val FAR_RETURN_START_M = 6.0f
    private const val FAR_SUPPORT_ANGLE_DEG = 3.0f
    private const val FAR_SUPPORT_BASE_M = 0.16f
    private const val FAR_SUPPORT_SPACING_MULTIPLIER = 2.0f
    private const val MIN_FAR_SUPPORT_COUNT = 2
    private val FAR_SUPPORT_OFFSETS = intArrayOf(-2, -1, 1, 2)
}
