package com.cartographer.demo

import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.SystemClock
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.ln
import kotlin.math.max
import kotlin.math.sin
import kotlin.math.sqrt

data class MagneticHeadingState(
    val headingRadians: Float = 0f,
    val accuracy: Int = SensorManager.SENSOR_STATUS_UNRELIABLE,
    val sampleCount: Int = 0,
    val dispersionDegrees: Float = Float.POSITIVE_INFINITY,
    val timestampNs: Long = 0L,
    val isStable: Boolean = false,
    val isSupported: Boolean = false,
    val isRegistered: Boolean = false,
    val sensorName: String = "--",
    val sensorVendor: String = "--",
    val callbackCount: Long = 0L,
    val magneticXMicroTesla: Float = 0f,
    val magneticYMicroTesla: Float = 0f,
    val magneticZMicroTesla: Float = 0f,
    val fieldStrengthMicroTesla: Float = 0f,
    val fieldStrengthStdDevMicroTesla: Float = Float.POSITIVE_INFINITY,
    val samplingDurationMillis: Long = 0L
)

class MagneticHeadingProvider(
    private val sensorManager: SensorManager
) : SensorEventListener {
    private val lock = Any()
    private val gravity = FloatArray(3)
    private val geomagnetic = FloatArray(3)
    private val rotationMatrix = FloatArray(9)
    private val orientation = FloatArray(3)
    private val recentHeadings = ArrayDeque<Float>(MAX_HEADING_SAMPLES)
    private val recentFieldStrengths = ArrayDeque<Float>(MAX_HEADING_SAMPLES)
    private val accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
    private val magnetometer = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)
    private var hasGravity = false
    private var hasGeomagnetic = false
    private var magneticAccuracy = SensorManager.SENSOR_STATUS_UNRELIABLE
    private var latestTimestampNs = 0L
    private var firstSampleTimestampNs = 0L
    @Volatile private var running = false
    private var magneticCallbackCount = 0L
    private var magneticX = 0f
    private var magneticY = 0f
    private var magneticZ = 0f
    private var fieldStrength = 0f

    val isSupported: Boolean get() = accelerometer != null && magnetometer != null

    fun start(): Boolean {
        if (!isSupported) return false
        if (running) return true
        val accelerometerRegistered = sensorManager.registerListener(
            this, accelerometer, SensorManager.SENSOR_DELAY_GAME
        )
        val magnetometerRegistered = sensorManager.registerListener(
            this, magnetometer, SensorManager.SENSOR_DELAY_GAME
        )
        running = accelerometerRegistered && magnetometerRegistered
        if (!running) sensorManager.unregisterListener(this)
        return running
    }

    fun stop() {
        sensorManager.unregisterListener(this)
        running = false
    }

    fun beginCalibration() {
        synchronized(lock) {
            recentHeadings.clear()
            recentFieldStrengths.clear()
            latestTimestampNs = 0L
            firstSampleTimestampNs = 0L
        }
    }

    fun getState(): MagneticHeadingState = synchronized(lock) {
        if (recentHeadings.isEmpty()) return@synchronized baseState()
        var sinSum = 0.0
        var cosSum = 0.0
        for (heading in recentHeadings) {
            sinSum += sin(heading.toDouble())
            cosSum += cos(heading.toDouble())
        }
        val count = recentHeadings.size
        val mean = normalizeRadians(atan2(sinSum, cosSum)).toFloat()
        val resultantLength = (sqrt(sinSum * sinSum + cosSum * cosSum) / count)
            .coerceIn(1e-6, 1.0)
        val dispersionDegrees = Math.toDegrees(
            sqrt(max(0.0, -2.0 * ln(resultantLength)))
        ).toFloat()
        val fieldMean = recentFieldStrengths.average()
        val fieldVariance = recentFieldStrengths.sumOf {
            val delta = it.toDouble() - fieldMean
            delta * delta
        } / recentFieldStrengths.size.coerceAtLeast(1)
        val fieldStdDev = sqrt(fieldVariance).toFloat()
        val samplingDurationMillis = if (
            firstSampleTimestampNs > 0L && latestTimestampNs >= firstSampleTimestampNs
        ) {
            (latestTimestampNs - firstSampleTimestampNs) / 1_000_000L
        } else {
            0L
        }
        val fresh = latestTimestampNs > 0L &&
            SystemClock.elapsedRealtimeNanos() - latestTimestampNs <= MAX_SAMPLE_AGE_NS
        MagneticHeadingState(
            headingRadians = mean,
            accuracy = magneticAccuracy,
            sampleCount = count,
            dispersionDegrees = dispersionDegrees,
            timestampNs = latestTimestampNs,
            isStable = fresh && count >= MIN_STABLE_SAMPLES &&
                samplingDurationMillis >= MIN_STABLE_DURATION_MS &&
                fieldStrength in MIN_FIELD_STRENGTH_UT..MAX_FIELD_STRENGTH_UT &&
                fieldStdDev <= MAX_FIELD_STRENGTH_STD_DEV_UT &&
                dispersionDegrees <= MAX_STABLE_DISPERSION_DEG,
            isSupported = isSupported,
            isRegistered = running,
            sensorName = magnetometer?.name ?: "--",
            sensorVendor = magnetometer?.vendor ?: "--",
            callbackCount = magneticCallbackCount,
            magneticXMicroTesla = magneticX,
            magneticYMicroTesla = magneticY,
            magneticZMicroTesla = magneticZ,
            fieldStrengthMicroTesla = fieldStrength,
            fieldStrengthStdDevMicroTesla = fieldStdDev,
            samplingDurationMillis = samplingDurationMillis
        )
    }

    private fun baseState() = MagneticHeadingState(
        accuracy = magneticAccuracy,
        timestampNs = latestTimestampNs,
        isSupported = isSupported,
        isRegistered = running,
        sensorName = magnetometer?.name ?: "--",
        sensorVendor = magnetometer?.vendor ?: "--",
        callbackCount = magneticCallbackCount,
        magneticXMicroTesla = magneticX,
        magneticYMicroTesla = magneticY,
        magneticZMicroTesla = magneticZ,
        fieldStrengthMicroTesla = fieldStrength
    )

    override fun onSensorChanged(event: SensorEvent) {
        synchronized(lock) {
            var magneticSampleUpdated = false
            when (event.sensor.type) {
                Sensor.TYPE_ACCELEROMETER -> {
                    if (!hasGravity) {
                        event.values.copyInto(gravity, endIndex = minOf(3, event.values.size))
                    } else {
                        for (index in 0 until minOf(3, event.values.size)) {
                            gravity[index] += SENSOR_LOW_PASS_ALPHA *
                                (event.values[index] - gravity[index])
                        }
                    }
                    hasGravity = true
                }
                Sensor.TYPE_MAGNETIC_FIELD -> {
                    magneticX = event.values[0]
                    magneticY = event.values[1]
                    magneticZ = event.values[2]
                    if (!hasGeomagnetic) {
                        event.values.copyInto(
                            geomagnetic,
                            endIndex = minOf(3, event.values.size)
                        )
                    } else {
                        for (index in 0 until minOf(3, event.values.size)) {
                            geomagnetic[index] += SENSOR_LOW_PASS_ALPHA *
                                (event.values[index] - geomagnetic[index])
                        }
                    }
                    hasGeomagnetic = true
                    fieldStrength = sqrt(
                        magneticX * magneticX +
                            magneticY * magneticY +
                            magneticZ * magneticZ
                    )
                    magneticCallbackCount++
                    magneticSampleUpdated = true
                }
                else -> return
            }
            if (!magneticSampleUpdated || !hasGravity || !hasGeomagnetic ||
                !SensorManager.getRotationMatrix(rotationMatrix, null, gravity, geomagnetic)) {
                return
            }
            SensorManager.getOrientation(rotationMatrix, orientation)
            recentHeadings.addLast(normalizeRadians(orientation[0].toDouble()).toFloat())
            recentFieldStrengths.addLast(fieldStrength)
            while (recentHeadings.size > MAX_HEADING_SAMPLES) {
                recentHeadings.removeFirst()
                recentFieldStrengths.removeFirst()
            }
            if (firstSampleTimestampNs == 0L) firstSampleTimestampNs = event.timestamp
            latestTimestampNs = event.timestamp
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
        if (sensor?.type == Sensor.TYPE_MAGNETIC_FIELD) {
            synchronized(lock) { magneticAccuracy = accuracy }
        }
    }

    private fun normalizeRadians(value: Double): Double {
        var normalized = value % (2.0 * PI)
        if (normalized < 0.0) normalized += 2.0 * PI
        return normalized
    }

    private companion object {
        private const val MAX_HEADING_SAMPLES = 512
        private const val MIN_STABLE_SAMPLES = 25
        private const val MIN_STABLE_DURATION_MS = 1_500L
        private const val MAX_STABLE_DISPERSION_DEG = 12f
        private const val MAX_SAMPLE_AGE_NS = 1_000_000_000L
        private const val MIN_FIELD_STRENGTH_UT = 15f
        private const val MAX_FIELD_STRENGTH_UT = 100f
        private const val MAX_FIELD_STRENGTH_STD_DEV_UT = 6f
        private const val SENSOR_LOW_PASS_ALPHA = 0.12f
    }
}
