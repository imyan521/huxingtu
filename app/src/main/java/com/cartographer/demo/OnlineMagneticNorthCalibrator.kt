package com.cartographer.demo

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.floor
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.sin
import kotlin.math.sqrt

/** Estimates fixed 3D magnetic bias and magnetic north while SLAM is running. */
class OnlineMagneticNorthCalibrator {
    data class Result(
        val isReady: Boolean = false,
        val magneticNorthYawRadians: Float = 0f,
        val biasRightMicroTesla: Float = 0f,
        val biasForwardMicroTesla: Float = 0f,
        val horizontalFieldMicroTesla: Float = 0f,
        val sampleCount: Int = 0,
        val inlierCount: Int = 0,
        val headingCoverageDegrees: Float = 0f,
        val residualMicroTesla: Float = Float.POSITIVE_INFINITY,
        val isSaturated: Boolean = false,
        val isHighInterference: Boolean = false,
        val usesThreeDimensionalBias: Boolean = false,
        val failureReason: String = "等待建图数据"
    )

    private data class Basis(
        val rightX: Double,
        val rightY: Double,
        val rightZ: Double,
        val forwardX: Double,
        val forwardY: Double,
        val forwardZ: Double,
        val downX: Double,
        val downY: Double,
        val downZ: Double
    )

    private data class Sample(
        val timestampNs: Long,
        val magneticX: Double,
        val magneticY: Double,
        val magneticZ: Double,
        val basis: Basis,
        val mapX: Double,
        val mapY: Double,
        val mapYaw: Double
    ) {
        val right: Double get() = magneticX * basis.rightX +
            magneticY * basis.rightY + magneticZ * basis.rightZ
        val forward: Double get() = magneticX * basis.forwardX +
            magneticY * basis.forwardY + magneticZ * basis.forwardZ
    }

    private data class Fit(
        val coefficients: DoubleArray,
        val residual: Double,
        val samples: List<Sample>,
        val isThreeDimensional: Boolean
    ) {
        val biasRight: Double get() = coefficients[0]
        val biasForward: Double get() = coefficients[1]
        val fieldX: Double get() = if (isThreeDimensional) coefficients[3] else coefficients[2]
        val fieldY: Double get() = if (isThreeDimensional) coefficients[4] else coefficients[3]
    }

    private val samples = ArrayList<Sample>(MAX_SAMPLES)
    private var lastTimestampNs = 0L
    private var saturatedSamples = 0
    private var lastReadyResult: Result? = null

    @Synchronized
    fun reset() {
        samples.clear()
        lastTimestampNs = 0L
        saturatedSamples = 0
        lastReadyResult = null
    }

    @Synchronized
    fun addSample(state: MagneticHeadingState, pose: Pose2D) {
        if (state.timestampNs <= lastTimestampNs || !pose.theta.isFinite()) return
        lastTimestampNs = state.timestampNs
        val maximumRange = state.sensorMaximumRangeMicroTesla
        if (maximumRange.isFinite() && maximumRange > 0f &&
            maxOf(
                abs(state.magneticXMicroTesla),
                abs(state.magneticYMicroTesla),
                abs(state.magneticZMicroTesla)
            ) >= maximumRange * SATURATION_FRACTION) {
            saturatedSamples++
            return
        }
        val basis = horizontalBasis(state) ?: return
        if (samples.size == MAX_SAMPLES) samples.removeAt(0)
        samples += Sample(
            timestampNs = state.timestampNs,
            magneticX = state.magneticXMicroTesla.toDouble(),
            magneticY = state.magneticYMicroTesla.toDouble(),
            magneticZ = state.magneticZMicroTesla.toDouble(),
            basis = basis,
            mapX = pose.x,
            mapY = pose.y,
            mapYaw = normalizeRadians(pose.theta)
        )
    }

    @Synchronized
    fun getResult(optimizedNodes: List<TrajectoryNodePose2D> = emptyList()): Result {
        val result = calculateResult(optimizedNodes)
        if (optimizedNodes.isNotEmpty()) return result
        if (result.isReady) {
            lastReadyResult = result
            return result
        }
        return lastReadyResult?.copy(
            failureReason = "已保留建图过程中最近一次有效校准结果"
        ) ?: result
    }

    private fun calculateResult(optimizedNodes: List<TrajectoryNodePose2D>): Result {
        val saturated = saturatedSamples >= MIN_SATURATED_SAMPLES &&
            saturatedSamples * 4 >= max(1, saturatedSamples + samples.size)
        if (saturated) {
            return Result(
                sampleCount = samples.size,
                isSaturated = true,
                failureReason = "磁力计已达到硬件量程"
            )
        }
        val poseAdjusted = if (optimizedNodes.isEmpty()) {
            samples.toList()
        } else {
            samples.mapNotNull { sample ->
                optimizedYawAt(sample.timestampNs, optimizedNodes)?.let { yaw ->
                    sample.copy(mapYaw = yaw)
                }
            }
        }
        val balanced = balanceSamples(poseAdjusted)
        if (balanced.size < MIN_SAMPLES) {
            return Result(
                sampleCount = balanced.size,
                failureReason = if (optimizedNodes.isEmpty()) {
                    "有效磁场与位姿样本不足"
                } else {
                    "磁场样本无法与优化轨迹匹配"
                }
            )
        }
        val coverage = headingCoverageDegrees(balanced.map { it.mapYaw })
        val fit = robustFit(balanced) ?: return Result(
            sampleCount = balanced.size,
            headingCoverageDegrees = coverage,
            failureReason = "转向范围不足，无法分离雷达磁偏移"
        )
        val inlierCoverage = headingCoverageDegrees(fit.samples.map { it.mapYaw })
        val field = hypot(fit.fieldX, fit.fieldY)
        val warningResidualLimit = max(WARNING_ABSOLUTE_RESIDUAL_UT, field * WARNING_RELATIVE_RESIDUAL)
        val hardResidualLimit = max(HARD_ABSOLUTE_RESIDUAL_UT, field * HARD_RELATIVE_RESIDUAL)
        val inlierRatio = fit.samples.size.toDouble() / balanced.size
        val highInterference = fit.residual > warningResidualLimit
        val ready = fit.samples.size >= MIN_INLIERS &&
            inlierCoverage >= MIN_HEADING_COVERAGE_DEGREES &&
            inlierRatio >= MIN_INLIER_RATIO &&
            field in MIN_HORIZONTAL_FIELD_UT..MAX_HORIZONTAL_FIELD_UT &&
            fit.residual <= hardResidualLimit
        val reason = when {
            inlierCoverage < MIN_HEADING_COVERAGE_DEGREES ->
                "请继续建图并经过更多不同方向的转弯"
            fit.samples.size < MIN_INLIERS || inlierRatio < MIN_INLIER_RATIO ->
                "有效磁场样本不足，请继续走动"
            field < MIN_HORIZONTAL_FIELD_UT -> "扣除雷达磁偏移后地磁信号过弱"
            field > MAX_HORIZONTAL_FIELD_UT -> "环境磁场过强，请远离强磁干扰区域"
            fit.residual > hardResidualLimit -> "磁场变化过大，无法可靠计算磁北"
            highInterference -> "在线地磁校准完成，当前环境磁干扰较强"
            else -> "在线地磁校准完成"
        }
        return Result(
            isReady = ready,
            magneticNorthYawRadians = normalizeRadians(atan2(fit.fieldY, fit.fieldX)).toFloat(),
            biasRightMicroTesla = fit.biasRight.toFloat(),
            biasForwardMicroTesla = fit.biasForward.toFloat(),
            horizontalFieldMicroTesla = field.toFloat(),
            sampleCount = balanced.size,
            inlierCount = fit.samples.size,
            headingCoverageDegrees = inlierCoverage,
            residualMicroTesla = fit.residual.toFloat(),
            isHighInterference = highInterference,
            usesThreeDimensionalBias = fit.isThreeDimensional,
            failureReason = reason
        )
    }

    private fun balanceSamples(input: List<Sample>): List<Sample> {
        val counts = HashMap<String, Int>()
        val output = ArrayList<Sample>(input.size)
        for (sample in input) {
            val xBin = floor(sample.mapX / POSITION_BIN_METERS).toInt()
            val yBin = floor(sample.mapY / POSITION_BIN_METERS).toInt()
            val yawBin = floor(normalizeRadians(sample.mapYaw) / YAW_BIN_RADIANS).toInt()
            val key = "$xBin:$yBin:$yawBin"
            val count = counts[key] ?: 0
            if (count >= MAX_SAMPLES_PER_BIN) continue
            counts[key] = count + 1
            output += sample
        }
        return output
    }

    private fun robustFit(input: List<Sample>): Fit? {
        val threeDimensional = robustFitModel(input, true)
        return threeDimensional ?: robustFitModel(input, false)
    }

    private fun robustFitModel(input: List<Sample>, useThreeDimensions: Boolean): Fit? {
        var selected = input
        var coefficients = leastSquares(selected, useThreeDimensions) ?: return null
        repeat(3) {
            val residuals = selected.map {
                sampleResidual(it, coefficients, useThreeDimensions)
            }
            val sorted = residuals.sorted()
            val median = sorted[sorted.size / 2]
            val threshold = max(MIN_OUTLIER_THRESHOLD_UT, median * OUTLIER_MEDIAN_MULTIPLIER)
            val filtered = selected.filterIndexed { index, _ -> residuals[index] <= threshold }
            if (filtered.size < MIN_INLIERS) return@repeat
            selected = filtered
            coefficients = leastSquares(selected, useThreeDimensions) ?: return null
        }
        val rms = sqrt(selected.sumOf {
            val residual = sampleResidual(it, coefficients, useThreeDimensions)
            residual * residual
        } / selected.size)
        return Fit(coefficients, rms, selected, useThreeDimensions)
    }

    private fun leastSquares(input: List<Sample>, useThreeDimensions: Boolean): DoubleArray? {
        val size = if (useThreeDimensions) 6 else 4
        val normal = Array(size) { DoubleArray(size) }
        val rhs = DoubleArray(size)
        for (sample in input) {
            val sine = sin(sample.mapYaw)
            val cosine = cos(sample.mapYaw)
            if (useThreeDimensions) {
                val basis = sample.basis
                accumulateNormal(
                    normal, rhs,
                    doubleArrayOf(
                        1.0, 0.0, 0.0,
                        basis.rightX * sine + basis.forwardX * cosine,
                        -basis.rightX * cosine + basis.forwardX * sine,
                        basis.downX
                    ),
                    sample.magneticX
                )
                accumulateNormal(
                    normal, rhs,
                    doubleArrayOf(
                        0.0, 1.0, 0.0,
                        basis.rightY * sine + basis.forwardY * cosine,
                        -basis.rightY * cosine + basis.forwardY * sine,
                        basis.downY
                    ),
                    sample.magneticY
                )
                accumulateNormal(
                    normal, rhs,
                    doubleArrayOf(
                        0.0, 0.0, 1.0,
                        basis.rightZ * sine + basis.forwardZ * cosine,
                        -basis.rightZ * cosine + basis.forwardZ * sine,
                        basis.downZ
                    ),
                    sample.magneticZ
                )
            } else {
                accumulateNormal(
                    normal, rhs,
                    doubleArrayOf(1.0, 0.0, sine, -cosine),
                    sample.right
                )
                accumulateNormal(
                    normal, rhs,
                    doubleArrayOf(0.0, 1.0, cosine, sine),
                    sample.forward
                )
            }
        }
        return solve(normal, rhs)
    }

    private fun accumulateNormal(
        normal: Array<DoubleArray>,
        rhs: DoubleArray,
        row: DoubleArray,
        target: Double
    ) {
        for (i in row.indices) {
            rhs[i] += row[i] * target
            for (j in row.indices) normal[i][j] += row[i] * row[j]
        }
    }

    private fun sampleResidual(
        sample: Sample,
        coefficients: DoubleArray,
        useThreeDimensions: Boolean
    ): Double {
        val sine = sin(sample.mapYaw)
        val cosine = cos(sample.mapYaw)
        if (!useThreeDimensions) {
            val expectedRight = coefficients[0] + coefficients[2] * sine - coefficients[3] * cosine
            val expectedForward = coefficients[1] + coefficients[2] * cosine + coefficients[3] * sine
            return hypot(sample.right - expectedRight, sample.forward - expectedForward)
        }
        val basis = sample.basis
        val rightField = coefficients[3] * sine - coefficients[4] * cosine
        val forwardField = coefficients[3] * cosine + coefficients[4] * sine
        val downField = coefficients[5]
        val expectedX = coefficients[0] + basis.rightX * rightField +
            basis.forwardX * forwardField + basis.downX * downField
        val expectedY = coefficients[1] + basis.rightY * rightField +
            basis.forwardY * forwardField + basis.downY * downField
        val expectedZ = coefficients[2] + basis.rightZ * rightField +
            basis.forwardZ * forwardField + basis.downZ * downField
        val dx = sample.magneticX - expectedX
        val dy = sample.magneticY - expectedY
        val dz = sample.magneticZ - expectedZ
        return sqrt(dx * dx + dy * dy + dz * dz)
    }

    private fun solve(matrix: Array<DoubleArray>, rhs: DoubleArray): DoubleArray? {
        val size = rhs.size
        val augmented = Array(size) { row ->
            DoubleArray(size + 1) { column ->
                if (column == size) rhs[row] else matrix[row][column]
            }
        }
        var largestPivot = 0.0
        var smallestPivot = Double.POSITIVE_INFINITY
        for (column in 0 until size) {
            var pivot = column
            for (row in column + 1 until size) {
                if (abs(augmented[row][column]) > abs(augmented[pivot][column])) pivot = row
            }
            val pivotMagnitude = abs(augmented[pivot][column])
            if (!pivotMagnitude.isFinite() || pivotMagnitude < MIN_SOLVER_PIVOT) return null
            largestPivot = max(largestPivot, pivotMagnitude)
            smallestPivot = kotlin.math.min(smallestPivot, pivotMagnitude)
            if (pivot != column) {
                val temporary = augmented[pivot]
                augmented[pivot] = augmented[column]
                augmented[column] = temporary
            }
            val scale = augmented[column][column]
            for (index in column..size) augmented[column][index] /= scale
            for (row in 0 until size) {
                if (row == column) continue
                val factor = augmented[row][column]
                for (index in column..size) augmented[row][index] -= factor * augmented[column][index]
            }
        }
        if (largestPivot / smallestPivot > MAX_PIVOT_RATIO) return null
        return DoubleArray(size) { augmented[it][size] }
    }

    private fun optimizedYawAt(
        timestampNs: Long,
        nodes: List<TrajectoryNodePose2D>
    ): Double? {
        if (nodes.isEmpty()) return null
        var low = 0
        var high = nodes.lastIndex
        while (low <= high) {
            val middle = (low + high) ushr 1
            if (nodes[middle].timestampNs < timestampNs) low = middle + 1 else high = middle - 1
        }
        val next = nodes.getOrNull(low)
        val previous = nodes.getOrNull(low - 1)
        if (previous != null && next != null && next.timestampNs > previous.timestampNs &&
            next.timestampNs - previous.timestampNs <= MAX_NODE_TIME_GAP_NS * 2) {
            val ratio = ((timestampNs - previous.timestampNs).toDouble() /
                (next.timestampNs - previous.timestampNs).toDouble()).coerceIn(0.0, 1.0)
            val delta = normalizeSigned(next.theta - previous.theta)
            return normalizeRadians(previous.theta + delta * ratio)
        }
        val nearest = previous ?: next ?: return null
        return if (abs(timestampNs - nearest.timestampNs) <= MAX_NODE_TIME_GAP_NS) {
            normalizeRadians(nearest.theta)
        } else null
    }

    private fun headingCoverageDegrees(headings: List<Double>): Float {
        if (headings.size < 2) return 0f
        val sorted = headings.map(::normalizeRadians).sorted()
        var largestGap = 0.0
        for (index in 1 until sorted.size) {
            largestGap = max(largestGap, sorted[index] - sorted[index - 1])
        }
        largestGap = max(largestGap, sorted.first() + FULL_TURN - sorted.last())
        return Math.toDegrees(FULL_TURN - largestGap).toFloat().coerceIn(0f, 360f)
    }

    private fun horizontalBasis(state: MagneticHeadingState): Basis? {
        val magnitude = state.gravityMagnitude
        if (!magnitude.isFinite() || magnitude < 1f) return null
        val downX = state.gravityX / magnitude
        val downY = state.gravityY / magnitude
        val downZ = state.gravityZ / magnitude
        var forwardX = -downY * downX
        var forwardY = 1f - downY * downY
        var forwardZ = -downY * downZ
        val forwardLength = sqrt(
            forwardX * forwardX + forwardY * forwardY + forwardZ * forwardZ
        )
        if (!forwardLength.isFinite() || forwardLength < MIN_FORWARD_PROJECTION) return null
        forwardX /= forwardLength
        forwardY /= forwardLength
        forwardZ /= forwardLength
        return Basis(
            rightX = (forwardY * downZ - forwardZ * downY).toDouble(),
            rightY = (forwardZ * downX - forwardX * downZ).toDouble(),
            rightZ = (forwardX * downY - forwardY * downX).toDouble(),
            forwardX = forwardX.toDouble(),
            forwardY = forwardY.toDouble(),
            forwardZ = forwardZ.toDouble(),
            downX = downX.toDouble(),
            downY = downY.toDouble(),
            downZ = downZ.toDouble()
        )
    }

    private fun normalizeRadians(value: Double): Double {
        var normalized = value % FULL_TURN
        if (normalized < 0.0) normalized += FULL_TURN
        return normalized
    }

    private fun normalizeSigned(value: Double): Double {
        var normalized = value % FULL_TURN
        if (normalized > PI) normalized -= FULL_TURN
        if (normalized < -PI) normalized += FULL_TURN
        return normalized
    }

    private companion object {
        private const val FULL_TURN = 2.0 * PI
        private const val MAX_SAMPLES = 6000
        private const val MIN_SAMPLES = 30
        private const val MIN_INLIERS = 24
        private const val MIN_HEADING_COVERAGE_DEGREES = 90f
        private const val MIN_INLIER_RATIO = 0.40
        private const val MIN_HORIZONTAL_FIELD_UT = 5.0
        private const val MAX_HORIZONTAL_FIELD_UT = 120.0
        private const val WARNING_ABSOLUTE_RESIDUAL_UT = 15.0
        private const val WARNING_RELATIVE_RESIDUAL = 0.50
        private const val HARD_ABSOLUTE_RESIDUAL_UT = 35.0
        private const val HARD_RELATIVE_RESIDUAL = 1.25
        private const val MIN_OUTLIER_THRESHOLD_UT = 6.0
        private const val OUTLIER_MEDIAN_MULTIPLIER = 4.0
        private const val MIN_FORWARD_PROJECTION = 0.35f
        private const val SATURATION_FRACTION = 0.92f
        private const val MIN_SATURATED_SAMPLES = 5
        private const val POSITION_BIN_METERS = 0.50
        private val YAW_BIN_RADIANS = Math.toRadians(10.0)
        private const val MAX_SAMPLES_PER_BIN = 3
        private const val MAX_NODE_TIME_GAP_NS = 1_500_000_000L
        private const val MIN_SOLVER_PIVOT = 1e-8
        private const val MAX_PIVOT_RATIO = 1e8
    }
}
