package com.cartographer.demo

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Calibrates magnetic north before SLAM starts. The user rotates the whole
 * phone/lidar assembly once and returns to the calibration start pose. A fitted
 * circle removes the magnetic field fixed to the assembly. The fitted bias can
 * then be applied again at the exact moment mapping starts.
 */
class MagneticNorthCalibrator {
    data class Result(
        val isReady: Boolean = false,
        val correctedHeadingRadians: Float = 0f,
        val biasRightMicroTesla: Float = 0f,
        val biasForwardMicroTesla: Float = 0f,
        val sampleCount: Int = 0,
        val rotationDegrees: Float = 0f,
        val returnErrorDegrees: Float = 180f,
        val magneticCoverageDegrees: Float = 0f,
        val radialResidualMicroTesla: Float = Float.POSITIVE_INFINITY,
        val isSaturated: Boolean = false,
        val failureReason: String = "标定尚未开始"
    )

    private data class Sample(
        val right: Double,
        val forward: Double,
        val yawFromStart: Double
    )

    private data class Circle(
        val centerX: Double,
        val centerY: Double,
        val radius: Double,
        val residual: Double
    )

    private val samples = ArrayList<Sample>(MAX_SAMPLES)
    private var active = false
    private var lastGyroscopeTimestampNs = 0L
    private var lastMagneticTimestampNs = 0L
    private var yawFromStart = 0.0
    private var accumulatedRotation = 0.0
    private var saturatedSamples = 0

    @Synchronized
    fun begin() {
        samples.clear()
        active = true
        lastGyroscopeTimestampNs = 0L
        lastMagneticTimestampNs = 0L
        yawFromStart = 0.0
        accumulatedRotation = 0.0
        saturatedSamples = 0
    }

    @Synchronized
    fun cancel() {
        active = false
    }

    @Synchronized
    fun addGyroscopeSample(
        timestampNs: Long,
        gyroX: Float,
        gyroY: Float,
        gyroZ: Float,
        gravityX: Float,
        gravityY: Float,
        gravityZ: Float
    ) {
        if (!active || timestampNs <= lastGyroscopeTimestampNs) return
        val previousTimestamp = lastGyroscopeTimestampNs
        lastGyroscopeTimestampNs = timestampNs
        if (previousTimestamp == 0L) return
        val deltaSeconds = (timestampNs - previousTimestamp) / 1_000_000_000.0
        if (deltaSeconds !in 0.0..MAX_GYRO_INTERVAL_SECONDS) return
        val gravityMagnitude = sqrt(
            gravityX * gravityX + gravityY * gravityY + gravityZ * gravityZ
        )
        if (!gravityMagnitude.isFinite() || gravityMagnitude < 1f) return
        val yawRate = -(
            gyroX * gravityX + gyroY * gravityY + gyroZ * gravityZ
            ) / gravityMagnitude
        val deltaYaw = yawRate * deltaSeconds
        yawFromStart += deltaYaw
        accumulatedRotation += abs(deltaYaw)
    }

    @Synchronized
    fun addMagneticSample(state: MagneticHeadingState) {
        if (!active || state.timestampNs <= lastMagneticTimestampNs) return
        lastMagneticTimestampNs = state.timestampNs
        val maximumRange = state.sensorMaximumRangeMicroTesla
        if (maximumRange.isFinite() && maximumRange > 0f &&
            max(
                max(abs(state.magneticXMicroTesla), abs(state.magneticYMicroTesla)),
                abs(state.magneticZMicroTesla)
            ) >= maximumRange * SATURATION_FRACTION) {
            saturatedSamples++
            return
        }
        val projected = projectToHorizontal(state) ?: return
        if (samples.size == MAX_SAMPLES) samples.removeAt(0)
        samples += Sample(projected.first, projected.second, yawFromStart)
    }

    @Synchronized
    fun getResult(): Result {
        val rotationDegrees = Math.toDegrees(accumulatedRotation).toFloat()
        val returnErrorDegrees = Math.toDegrees(abs(normalizeSigned(yawFromStart))).toFloat()
        val saturated = saturatedSamples >= MIN_SATURATED_SAMPLES &&
            saturatedSamples * 4 >= max(1, saturatedSamples + samples.size)
        if (saturated) {
            return Result(
                sampleCount = samples.size,
                rotationDegrees = rotationDegrees,
                returnErrorDegrees = returnErrorDegrees,
                isSaturated = true,
                failureReason = "磁力计已达到硬件量程"
            )
        }
        if (samples.size < MIN_SAMPLES) {
            return Result(
                sampleCount = samples.size,
                rotationDegrees = rotationDegrees,
                returnErrorDegrees = returnErrorDegrees,
                failureReason = "有效磁场样本不足"
            )
        }
        val circle = robustCircleFit(samples) ?: return Result(
            sampleCount = samples.size,
            rotationDegrees = rotationDegrees,
            returnErrorDegrees = returnErrorDegrees,
            failureReason = "无法拟合固定磁偏置"
        )
        val correctedAngles = samples.map {
            normalizeRadians(atan2(-(it.right - circle.centerX), it.forward - circle.centerY))
        }
        val coverage = angularCoverageDegrees(correctedAngles)
        val anchorSamples = samples.filter {
            abs(normalizeSigned(it.yawFromStart)) <= ANCHOR_YAW_TOLERANCE_RAD
        }
        var sinSum = 0.0
        var cosSum = 0.0
        for (sample in anchorSamples) {
            val heading = atan2(
                -(sample.right - circle.centerX),
                sample.forward - circle.centerY
            )
            sinSum += sin(heading)
            cosSum += cos(heading)
        }
        val heading = normalizeRadians(atan2(sinSum, cosSum))
        val residualLimit = max(MAX_ABSOLUTE_RESIDUAL_UT, circle.radius * MAX_RELATIVE_RESIDUAL)
        val ready = accumulatedRotation >= MIN_ROTATION_RAD &&
            returnErrorDegrees <= MAX_RETURN_ERROR_DEGREES &&
            coverage >= MIN_MAGNETIC_COVERAGE_DEGREES &&
            anchorSamples.size >= MIN_ANCHOR_SAMPLES &&
            circle.radius >= MIN_FIELD_RADIUS_UT && circle.residual <= residualLimit
        val reason = when {
            rotationDegrees < MIN_ROTATION_DEGREES -> "请继续旋转整套设备"
            coverage < MIN_MAGNETIC_COVERAGE_DEGREES -> "磁场方向覆盖不足，请继续缓慢旋转"
            returnErrorDegrees > MAX_RETURN_ERROR_DEGREES -> "请转回开始校准时的朝向"
            anchorSamples.size < MIN_ANCHOR_SAMPLES -> "请在起始朝向短暂停留"
            circle.radius < MIN_FIELD_RADIUS_UT -> "扣除固定磁场后地磁信号过弱"
            circle.residual > residualLimit -> "旋转期间磁场变化过大，请保持设备水平并重试"
            else -> "标定完成"
        }
        return Result(
            isReady = ready,
            correctedHeadingRadians = heading.toFloat(),
            biasRightMicroTesla = circle.centerX.toFloat(),
            biasForwardMicroTesla = circle.centerY.toFloat(),
            sampleCount = samples.size,
            rotationDegrees = rotationDegrees,
            returnErrorDegrees = returnErrorDegrees,
            magneticCoverageDegrees = coverage,
            radialResidualMicroTesla = circle.residual.toFloat(),
            isSaturated = false,
            failureReason = reason
        )
    }

    /** Returns the current corrected heading using a completed calibration. */
    fun correctedHeadingRadians(
        state: MagneticHeadingState,
        calibration: Result
    ): Float? {
        if (!calibration.isReady) return null
        val projected = projectToHorizontal(state) ?: return null
        val right = projected.first - calibration.biasRightMicroTesla
        val forward = projected.second - calibration.biasForwardMicroTesla
        if (!right.isFinite() || !forward.isFinite() ||
            hypot(right, forward) < MIN_FIELD_RADIUS_UT) {
            return null
        }
        return normalizeRadians(atan2(-right, forward)).toFloat()
    }

    private fun projectToHorizontal(state: MagneticHeadingState): Pair<Double, Double>? {
        val gravityMagnitude = state.gravityMagnitude
        if (!gravityMagnitude.isFinite() || gravityMagnitude < 1f) return null
        val gx = state.gravityX / gravityMagnitude
        val gy = state.gravityY / gravityMagnitude
        val gz = state.gravityZ / gravityMagnitude
        var forwardX = -gy * gx
        var forwardY = 1f - gy * gy
        var forwardZ = -gy * gz
        val forwardLength = sqrt(
            forwardX * forwardX + forwardY * forwardY + forwardZ * forwardZ
        )
        if (!forwardLength.isFinite() || forwardLength < MIN_FORWARD_PROJECTION) return null
        forwardX /= forwardLength
        forwardY /= forwardLength
        forwardZ /= forwardLength
        val rightX = forwardY * gz - forwardZ * gy
        val rightY = forwardZ * gx - forwardX * gz
        val rightZ = forwardX * gy - forwardY * gx
        val right = state.magneticXMicroTesla * rightX +
            state.magneticYMicroTesla * rightY + state.magneticZMicroTesla * rightZ
        val forward = state.magneticXMicroTesla * forwardX +
            state.magneticYMicroTesla * forwardY + state.magneticZMicroTesla * forwardZ
        return right.toDouble() to forward.toDouble()
    }

    private fun robustCircleFit(input: List<Sample>): Circle? {
        var selected = input
        var circle = circleFit(selected) ?: return null
        repeat(2) {
            val deviations = selected.map {
                abs(hypot(it.right - circle.centerX, it.forward - circle.centerY) - circle.radius)
            }.sorted()
            val medianDeviation = deviations[deviations.size / 2]
            val threshold = max(MIN_OUTLIER_THRESHOLD_UT, medianDeviation * 4.0)
            val filtered = selected.filter {
                abs(hypot(it.right - circle.centerX, it.forward - circle.centerY) - circle.radius) <=
                    threshold
            }
            if (filtered.size < MIN_SAMPLES / 2) return@repeat
            selected = filtered
            circle = circleFit(selected) ?: return null
        }
        return circle
    }

    private fun circleFit(input: List<Sample>): Circle? {
        val normal = Array(3) { DoubleArray(3) }
        val rhs = DoubleArray(3)
        for (sample in input) {
            val row = doubleArrayOf(sample.right, sample.forward, 1.0)
            val target = -(sample.right * sample.right + sample.forward * sample.forward)
            for (i in 0..2) {
                rhs[i] += row[i] * target
                for (j in 0..2) normal[i][j] += row[i] * row[j]
            }
        }
        val solved = solve3x3(normal, rhs) ?: return null
        val centerX = -solved[0] * 0.5
        val centerY = -solved[1] * 0.5
        val radiusSquared = centerX * centerX + centerY * centerY - solved[2]
        if (!radiusSquared.isFinite() || radiusSquared <= 0.0) return null
        val radius = sqrt(radiusSquared)
        val residual = sqrt(input.sumOf {
            val difference = hypot(it.right - centerX, it.forward - centerY) - radius
            difference * difference
        } / input.size)
        return Circle(centerX, centerY, radius, residual)
    }

    private fun angularCoverageDegrees(angles: List<Double>): Float {
        val occupied = BooleanArray(COVERAGE_BIN_COUNT)
        for (angle in angles) {
            val index = ((normalizeRadians(angle) / FULL_TURN) * COVERAGE_BIN_COUNT)
                .toInt().coerceIn(0, COVERAGE_BIN_COUNT - 1)
            occupied[index] = true
        }
        return occupied.count { it } * (360f / COVERAGE_BIN_COUNT)
    }

    private fun solve3x3(matrix: Array<DoubleArray>, rhs: DoubleArray): DoubleArray? {
        val augmented = Array(3) { row ->
            doubleArrayOf(matrix[row][0], matrix[row][1], matrix[row][2], rhs[row])
        }
        for (column in 0..2) {
            var pivot = column
            for (row in column + 1..2) {
                if (abs(augmented[row][column]) > abs(augmented[pivot][column])) pivot = row
            }
            if (abs(augmented[pivot][column]) < 1e-8) return null
            if (pivot != column) {
                val temporary = augmented[pivot]
                augmented[pivot] = augmented[column]
                augmented[column] = temporary
            }
            val scale = augmented[column][column]
            for (index in column..3) augmented[column][index] /= scale
            for (row in 0..2) {
                if (row == column) continue
                val factor = augmented[row][column]
                for (index in column..3) {
                    augmented[row][index] -= factor * augmented[column][index]
                }
            }
        }
        return doubleArrayOf(augmented[0][3], augmented[1][3], augmented[2][3])
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
        private const val MAX_SAMPLES = 2500
        private const val MIN_SAMPLES = 80
        private const val MIN_ANCHOR_SAMPLES = 8
        private const val MIN_ROTATION_DEGREES = 330f
        private val MIN_ROTATION_RAD = Math.toRadians(MIN_ROTATION_DEGREES.toDouble())
        private const val MAX_RETURN_ERROR_DEGREES = 12f
        private val ANCHOR_YAW_TOLERANCE_RAD = Math.toRadians(15.0)
        private const val MIN_MAGNETIC_COVERAGE_DEGREES = 300f
        private const val MIN_FIELD_RADIUS_UT = 5.0
        private const val MAX_ABSOLUTE_RESIDUAL_UT = 18.0
        private const val MAX_RELATIVE_RESIDUAL = 0.35
        private const val MIN_OUTLIER_THRESHOLD_UT = 4.0
        private const val COVERAGE_BIN_COUNT = 24
        private const val MAX_GYRO_INTERVAL_SECONDS = 0.2
        private const val MIN_FORWARD_PROJECTION = 0.35f
        private const val SATURATION_FRACTION = 0.92f
        private const val MIN_SATURATED_SAMPLES = 5
    }
}
