package com.cartographer.demo

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

class OnlineMagneticNorthCalibratorTest {
    @Test
    fun estimatesNorthAndLargeFixedMagnetWhileMapping() {
        val calibrator = OnlineMagneticNorthCalibrator()
        val northYaw = Math.toRadians(52.0)
        val biasRight = 220.0
        val biasForward = -140.0
        val field = 36.0
        var timestamp = 1_000_000_000L

        for (degrees in 0..300 step 2) {
            val yaw = Math.toRadians(degrees.toDouble())
            val noise = sin(degrees * 0.41) * 0.45
            val right = biasRight + field * sin(yaw) * cos(northYaw) -
                field * cos(yaw) * sin(northYaw) + noise
            val forward = biasForward + field * cos(yaw) * cos(northYaw) +
                field * sin(yaw) * sin(northYaw) - noise * 0.3
            val disturbed = degrees % 34 == 0
            calibrator.addSample(
                magneticState(
                    timestamp,
                    (right + if (disturbed) 55.0 else 0.0).toFloat(),
                    (forward - if (disturbed) 45.0 else 0.0).toFloat()
                ),
                Pose2D(degrees / 20.0, degrees / 35.0, yaw)
            )
            timestamp += 100_000_000L
        }

        val result = calibrator.getResult()
        assertTrue(result.failureReason, result.isReady)
        assertEquals(biasRight.toFloat(), result.biasRightMicroTesla, 2.0f)
        assertEquals(biasForward.toFloat(), result.biasForwardMicroTesla, 2.0f)
        assertAngleEquals(northYaw, result.magneticNorthYawRadians.toDouble(), Math.toRadians(2.0))
        assertTrue(result.inlierCount < result.sampleCount)
    }

    @Test
    fun rejectsStraightTrajectoryWithoutHeadingCoverage() {
        val calibrator = OnlineMagneticNorthCalibrator()
        var timestamp = 1_000_000_000L
        repeat(120) { index ->
            calibrator.addSample(
                magneticState(timestamp, 180f, -105f),
                Pose2D(index * 0.1, 0.0, Math.toRadians(5.0))
            )
            timestamp += 100_000_000L
        }

        val result = calibrator.getResult()
        assertFalse(result.isReady)
        assertTrue(result.headingCoverageDegrees < 10f)
    }

    @Test
    fun acceptsNormalCornerWithoutRequiringHalfTurn() {
        val calibrator = OnlineMagneticNorthCalibrator()
        val northYaw = Math.toRadians(28.0)
        var timestamp = 1_000_000_000L
        for (degrees in 0..110 step 2) {
            val yaw = Math.toRadians(degrees.toDouble())
            val right = 210.0 + 35.0 * sin(yaw - northYaw)
            val forward = -130.0 + 35.0 * cos(yaw - northYaw)
            calibrator.addSample(
                magneticState(timestamp, right.toFloat(), forward.toFloat()),
                Pose2D(degrees / 15.0, degrees / 30.0, yaw)
            )
            timestamp += 100_000_000L
        }

        val result = calibrator.getResult()
        assertTrue(result.failureReason, result.isReady)
        assertTrue(result.headingCoverageDegrees >= 90f)
        assertAngleEquals(northYaw, result.magneticNorthYawRadians.toDouble(), Math.toRadians(1.0))
    }

    @Test
    fun retainsLastUsableResultDuringLaterSensorSaturation() {
        val calibrator = OnlineMagneticNorthCalibrator()
        var timestamp = 1_000_000_000L
        for (degrees in 0..110 step 2) {
            val yaw = Math.toRadians(degrees.toDouble())
            calibrator.addSample(
                magneticState(
                    timestamp,
                    (180.0 + 34.0 * sin(yaw)).toFloat(),
                    (-120.0 + 34.0 * cos(yaw)).toFloat()
                ),
                Pose2D(degrees / 15.0, 0.0, yaw)
            )
            timestamp += 100_000_000L
        }
        assertTrue(calibrator.getResult().isReady)

        repeat(30) { index ->
            calibrator.addSample(
                magneticState(timestamp, 1_950f, 1_950f),
                Pose2D(8.0 + index * 0.1, 0.0, Math.toRadians(110.0))
            )
            timestamp += 100_000_000L
        }

        val retained = calibrator.getResult()
        assertTrue(retained.failureReason, retained.isReady)
        assertTrue(retained.failureReason.contains("保留"))
    }

    @Test
    fun refitsNorthWithFinalOptimizedTrajectoryYaws() {
        val calibrator = OnlineMagneticNorthCalibrator()
        val liveNorthYaw = Math.toRadians(35.0)
        val optimizationRotation = Math.toRadians(14.0)
        val nodes = ArrayList<TrajectoryNodePose2D>()
        var timestamp = 2_000_000_000L

        for (degrees in 0..300 step 3) {
            val liveYaw = Math.toRadians(degrees.toDouble())
            val field = 32.0
            val right = 190.0 + field * sin(liveYaw) * cos(liveNorthYaw) -
                field * cos(liveYaw) * sin(liveNorthYaw)
            val forward = -125.0 + field * cos(liveYaw) * cos(liveNorthYaw) +
                field * sin(liveYaw) * sin(liveNorthYaw)
            calibrator.addSample(
                magneticState(timestamp, right.toFloat(), forward.toFloat()),
                Pose2D(degrees / 30.0, 0.0, liveYaw)
            )
            nodes += TrajectoryNodePose2D(
                timestampNs = timestamp,
                x = degrees / 30.0,
                y = 0.0,
                theta = liveYaw + optimizationRotation
            )
            timestamp += 100_000_000L
        }

        val result = calibrator.getResult(nodes)
        assertTrue(result.failureReason, result.isReady)
        assertAngleEquals(
            liveNorthYaw + optimizationRotation,
            result.magneticNorthYawRadians.toDouble(),
            Math.toRadians(1.0)
        )
    }

    @Test
    fun threeDimensionalBiasRemainsStableWhileDeviceTilts() {
        val calibrator = OnlineMagneticNorthCalibrator()
        val northYaw = Math.toRadians(68.0)
        val bias = doubleArrayOf(235.0, -165.0, 110.0)
        var timestamp = 3_000_000_000L

        for (degrees in 0..300 step 2) {
            val yaw = Math.toRadians(degrees.toDouble())
            val gravityX = sin(degrees * 0.11) * 2.4
            val gravityY = cos(degrees * 0.07) * 1.8
            val gravityZ = sqrt(9.81 * 9.81 - gravityX * gravityX - gravityY * gravityY)
            calibrator.addSample(
                tiltedMagneticState(
                    timestamp = timestamp,
                    yaw = yaw,
                    northYaw = northYaw,
                    bias = bias,
                    gravityX = gravityX,
                    gravityY = gravityY,
                    gravityZ = gravityZ
                ),
                Pose2D(degrees / 25.0, degrees / 40.0, yaw)
            )
            timestamp += 100_000_000L
        }

        val result = calibrator.getResult()
        assertTrue(result.failureReason, result.isReady)
        assertTrue(result.usesThreeDimensionalBias)
        assertAngleEquals(northYaw, result.magneticNorthYawRadians.toDouble(), Math.toRadians(1.0))
        assertTrue(result.residualMicroTesla < 1f)
    }

    private fun magneticState(timestamp: Long, x: Float, y: Float) = MagneticHeadingState(
        timestampNs = timestamp,
        magneticXMicroTesla = x,
        magneticYMicroTesla = y,
        magneticZMicroTesla = 42f,
        gravityX = 0f,
        gravityY = 0f,
        gravityZ = 9.81f,
        gravityMagnitude = 9.81f,
        sensorMaximumRangeMicroTesla = 2_000f
    )

    private fun tiltedMagneticState(
        timestamp: Long,
        yaw: Double,
        northYaw: Double,
        bias: DoubleArray,
        gravityX: Double,
        gravityY: Double,
        gravityZ: Double
    ): MagneticHeadingState {
        val magnitude = sqrt(
            gravityX * gravityX + gravityY * gravityY + gravityZ * gravityZ
        )
        val downX = gravityX / magnitude
        val downY = gravityY / magnitude
        val downZ = gravityZ / magnitude
        var forwardX = -downY * downX
        var forwardY = 1.0 - downY * downY
        var forwardZ = -downY * downZ
        val forwardLength = sqrt(
            forwardX * forwardX + forwardY * forwardY + forwardZ * forwardZ
        )
        forwardX /= forwardLength
        forwardY /= forwardLength
        forwardZ /= forwardLength
        val rightX = forwardY * downZ - forwardZ * downY
        val rightY = forwardZ * downX - forwardX * downZ
        val rightZ = forwardX * downY - forwardY * downX
        val horizontalField = 38.0
        val rightField = horizontalField * sin(yaw - northYaw)
        val forwardField = horizontalField * cos(yaw - northYaw)
        val downField = 31.0
        return MagneticHeadingState(
            timestampNs = timestamp,
            magneticXMicroTesla = (bias[0] + rightX * rightField +
                forwardX * forwardField + downX * downField).toFloat(),
            magneticYMicroTesla = (bias[1] + rightY * rightField +
                forwardY * forwardField + downY * downField).toFloat(),
            magneticZMicroTesla = (bias[2] + rightZ * rightField +
                forwardZ * forwardField + downZ * downField).toFloat(),
            gravityX = gravityX.toFloat(),
            gravityY = gravityY.toFloat(),
            gravityZ = gravityZ.toFloat(),
            gravityMagnitude = magnitude.toFloat(),
            sensorMaximumRangeMicroTesla = 2_000f
        )
    }

    private fun assertAngleEquals(expected: Double, actual: Double, tolerance: Double) {
        var difference = (actual - expected) % (2.0 * PI)
        if (difference > PI) difference -= 2.0 * PI
        if (difference < -PI) difference += 2.0 * PI
        assertEquals(0.0, difference, tolerance)
    }
}
