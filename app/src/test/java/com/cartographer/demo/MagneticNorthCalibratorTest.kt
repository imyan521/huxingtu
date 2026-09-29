package com.cartographer.demo

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

class MagneticNorthCalibratorTest {
    @Test
    fun removesFixedMagnetAfterFullTurnAndReturn() {
        val calibrator = MagneticNorthCalibrator()
        calibrator.begin()
        var timestamp = 1_000_000_000L
        calibrator.addGyroscopeSample(timestamp, 0f, 0f, 0f, 0f, 0f, 9.81f)
        val startHeading = Math.toRadians(38.0)
        val stepSeconds = 0.04
        val stepRadians = Math.toRadians(2.0)
        for (degrees in 0..360 step 2) {
            val yaw = Math.toRadians(degrees.toDouble())
            timestamp += (stepSeconds * 1_000_000_000L).toLong()
            calibrator.addGyroscopeSample(
                timestamp,
                0f,
                0f,
                (-stepRadians / stepSeconds).toFloat(),
                0f,
                0f,
                9.81f
            )
            val heading = startHeading + yaw
            val noise = sin(degrees * 0.37) * 0.6
            calibrator.addMagneticSample(
                magneticState(
                    timestamp = timestamp,
                    x = (245.0 - 36.0 * sin(heading) + noise).toFloat(),
                    y = (-155.0 + 36.0 * cos(heading) - noise * 0.4).toFloat()
                )
            )
        }

        val result = calibrator.getResult()
        assertTrue(result.failureReason, result.isReady)
        assertEquals(245f, result.biasRightMicroTesla, 1.5f)
        assertEquals(-155f, result.biasForwardMicroTesla, 1.5f)
        assertAngleEquals(startHeading, result.correctedHeadingRadians.toDouble(), Math.toRadians(2.0))
    }

    @Test
    fun requiresReturnToCalibrationStartHeading() {
        val calibrator = MagneticNorthCalibrator()
        calibrator.begin()
        var timestamp = 1_000_000_000L
        calibrator.addGyroscopeSample(timestamp, 0f, 0f, 0f, 0f, 0f, 9.81f)
        val stepSeconds = 0.04
        val stepRadians = Math.toRadians(2.0)
        for (degrees in 0..340 step 2) {
            timestamp += (stepSeconds * 1_000_000_000L).toLong()
            calibrator.addGyroscopeSample(
                timestamp, 0f, 0f, (-stepRadians / stepSeconds).toFloat(), 0f, 0f, 9.81f
            )
            val heading = Math.toRadians(degrees.toDouble())
            calibrator.addMagneticSample(
                magneticState(
                    timestamp,
                    (180.0 - 32.0 * sin(heading)).toFloat(),
                    (-120.0 + 32.0 * cos(heading)).toFloat()
                )
            )
        }
        val result = calibrator.getResult()
        assertTrue(!result.isReady)
        assertTrue(result.returnErrorDegrees > 12f)
    }

    @Test
    fun appliesSavedBiasAtMappingStartHeading() {
        val calibrator = MagneticNorthCalibrator()
        calibrator.begin()
        var timestamp = 1_000_000_000L
        calibrator.addGyroscopeSample(timestamp, 0f, 0f, 0f, 0f, 0f, 9.81f)
        val stepSeconds = 0.04
        val stepRadians = Math.toRadians(2.0)
        for (degrees in 0..360 step 2) {
            val yaw = Math.toRadians(degrees.toDouble())
            timestamp += (stepSeconds * 1_000_000_000L).toLong()
            calibrator.addGyroscopeSample(
                timestamp, 0f, 0f, (-stepRadians / stepSeconds).toFloat(), 0f, 0f, 9.81f
            )
            calibrator.addMagneticSample(
                magneticState(
                    timestamp,
                    (220.0 - 34.0 * sin(yaw)).toFloat(),
                    (-140.0 + 34.0 * cos(yaw)).toFloat()
                )
            )
        }
        val calibration = calibrator.getResult()
        assertTrue(calibration.failureReason, calibration.isReady)

        val mappingStartHeading = Math.toRadians(117.0)
        val currentState = magneticState(
            timestamp + 1,
            (220.0 - 34.0 * sin(mappingStartHeading)).toFloat(),
            (-140.0 + 34.0 * cos(mappingStartHeading)).toFloat()
        )
        val corrected = calibrator.correctedHeadingRadians(currentState, calibration)

        assertTrue(corrected != null)
        assertAngleEquals(mappingStartHeading, corrected!!.toDouble(), Math.toRadians(1.0))
    }

    private fun magneticState(timestamp: Long, x: Float, y: Float) = MagneticHeadingState(
        timestampNs = timestamp,
        magneticXMicroTesla = x,
        magneticYMicroTesla = y,
        magneticZMicroTesla = 45f,
        gravityX = 0f,
        gravityY = 0f,
        gravityZ = 9.81f,
        gravityMagnitude = 9.81f,
        sensorMaximumRangeMicroTesla = 2_000f
    )

    private fun assertAngleEquals(expected: Double, actual: Double, tolerance: Double) {
        var difference = (actual - expected) % (2.0 * PI)
        if (difference > PI) difference -= 2.0 * PI
        if (difference < -PI) difference += 2.0 * PI
        assertEquals(0.0, difference, tolerance)
    }
}
