package com.cartographer.demo

import android.os.SystemClock
import android.util.Log
import kotlin.math.min

class LidarParser(private val carto: CartographerNative) {
    private val parserLock = Any()
    private val scanLock = Any()
    private val buffer = ByteArray(16 * 1024)
    private var bufferLength = 0
    private var lastTimeNs = SystemClock.elapsedRealtimeNanos()
    private var allowSendLidar = false

    private val scanRanges = ArrayList<Float>(720)
    private val scanAngles = ArrayList<Float>(720)
    // Includes samples without a usable distance return. Completeness is a
    // transport/protocol property and must not depend on whether a surface was
    // reflective enough to return a valid range.
    private val scanSampleAngles = ArrayList<Float>(720)
    private var lastRawAngleDeg: Float? = null
    private var waitingForScanBoundary = true
    private var liveScanAssemblyStartedAtBoundary = false
    private var scanStartNs = 0L
    private var lastCompletedScanEndNs = 0L
    // USB callbacks are delivery times, not lidar hardware times.  A delayed
    // callback must not make Cartographer deskew one revolution as if the
    // scanner had suddenly slowed down (that is especially damaging in turns).
    private var filteredScanDurationNs = DEFAULT_SCAN_DURATION_NS
    private var lastSendNs = 0L
    private var lastLogNs = 0L

    var lastValidPointCount = 0
    var lastShowDistance = 0f
    var lastShowAngle = 0f
    var parsedPacketCount = 0L
    var aa55PacketCount = 0L
    var invalidAa55PacketCount = 0L
    var droppedIncompleteScanCount = 0L
    var droppedSparseScanCount = 0L
    var droppedByteCount = 0L
    var rejectedOutlierPointCount = 0L
    var acceptedScanCount = 0L
    var lastScanCoverageDeg = 0f
    var lastScanMaximumGapDeg = 0f
    var activeProtocol = "AA55"
    private var latestScan: LidarScan? = null

    fun enableLidar() {
        allowSendLidar = true
    }

    // 保留原 IMU 模式接口，恢复 IMU 时 MainActivity 可重新调用该方法。
    fun enableLidarAfterImuReady() {
        allowSendLidar = true
    }

    fun getLatestScan(): LidarScan? = synchronized(scanLock) { latestScan }

    fun resetMappingSessionStatistics() {
        synchronized(parserLock) {
            droppedIncompleteScanCount = 0L
            droppedSparseScanCount = 0L
            rejectedOutlierPointCount = 0L
            acceptedScanCount = 0L
            lastScanCoverageDeg = 0f
            lastScanMaximumGapDeg = 0f
            lastValidPointCount = 0
        }
    }

    fun clearLatestScanAndPendingData() {
        synchronized(parserLock) {
            bufferLength = 0
            lastRawAngleDeg = null
            waitingForScanBoundary = true
            liveScanAssemblyStartedAtBoundary = false
            lastCompletedScanEndNs = 0L
            filteredScanDurationNs = DEFAULT_SCAN_DURATION_NS
            lastTimeNs = SystemClock.elapsedRealtimeNanos()
            clearCurrentScan()
            synchronized(scanLock) { latestScan = null }
        }
    }

    fun onDataReceived(data: ByteArray, length: Int) {
        synchronized(parserLock) {
            appendData(data, length)

            while (bufferLength >= MIN_HEADER_LENGTH) {
                val headIndex = findAa55Header()
                if (headIndex < 0) {
                    keepPossibleHeaderByte()
                    return
                }
                if (headIndex > 0) {
                    droppedByteCount += headIndex.toLong()
                    dropBytes(headIndex)
                }

                if (!consumeAa55Packet()) return
            }
        }
    }

    private fun consumeAa55Packet(): Boolean {
        if (bufferLength < AA55_HEADER_LENGTH) return false

        val pointCount = u8(buffer[3])
        if (pointCount <= 0 || pointCount > AA55_MAX_POINTS_PER_PACKET) {
            invalidAa55PacketCount++
            droppedByteCount++
            dropBytes(1)
            return true
        }

        // AA55 angle fields carry a protocol check bit in bit 0 of their low
        // byte. Reject a false header before trusting its point count/length;
        // otherwise random payload bytes can be expanded into a plausible but
        // geometrically destructive scan sector.
        if ((u8(buffer[4]) and AA55_ANGLE_CHECK_BIT) == 0 ||
            (u8(buffer[6]) and AA55_ANGLE_CHECK_BIT) == 0) {
            invalidAa55PacketCount++
            droppedByteCount++
            dropBytes(1)
            return true
        }

        val packetLength = AA55_HEADER_LENGTH + pointCount * BYTES_PER_POINT
        if (bufferLength < packetLength) return false

        if (!hasPlausiblePacketAngles(buffer, 0, pointCount)) {
            invalidAa55PacketCount++
            droppedByteCount += packetLength.toLong()
            dropBytes(packetLength)
            return true
        }

        parseAa55Packet(buffer, 0, pointCount)
        aa55PacketCount++
        parsedPacketCount++
        activeProtocol = "AA55"
        dropBytes(packetLength)
        return true
    }

    private fun hasPlausiblePacketAngles(
        packet: ByteArray,
        offset: Int,
        pointCount: Int
    ): Boolean {
        if (pointCount <= 1) return true
        val firstRaw = u8(packet[offset + 4]) or
            (u8(packet[offset + 5]) shl 8)
        val lastRaw = u8(packet[offset + 6]) or
            (u8(packet[offset + 7]) shl 8)
        val firstDeg = ((firstRaw shr 1) / 64.0f).normalizeAngle()
        val lastDeg = ((lastRaw shr 1) / 64.0f).normalizeAngle()
        val spanDeg = forwardAngleDelta(firstDeg, lastDeg)
        val stepDeg = spanDeg / (pointCount - 1)
        return spanDeg in MIN_PACKET_SPAN_DEG..MAX_PACKET_SPAN_DEG &&
            stepDeg in MIN_POINT_ANGLE_STEP_DEG..MAX_POINT_ANGLE_STEP_DEG
    }

    private fun appendData(data: ByteArray, length: Int) {
        if (length <= 0) return
        val copyLength = min(length, buffer.size)
        if (bufferLength + copyLength > buffer.size) {
            droppedByteCount += bufferLength.toLong()
            bufferLength = 0
        }
        System.arraycopy(data, length - copyLength, buffer, bufferLength, copyLength)
        bufferLength += copyLength
    }

    private fun findAa55Header(): Int {
        for (i in 0 until bufferLength - 1) {
            if (u8(buffer[i]) == 0xAA && u8(buffer[i + 1]) == 0x55) {
                return i
            }
        }
        return -1
    }

    private fun keepPossibleHeaderByte() {
        val keep = bufferLength > 0 && u8(buffer[bufferLength - 1]) == 0xAA
        droppedByteCount += if (keep) (bufferLength - 1).toLong() else bufferLength.toLong()
        if (keep) {
            buffer[0] = buffer[bufferLength - 1]
            bufferLength = 1
        } else {
            bufferLength = 0
        }
    }

    private fun dropBytes(count: Int) {
        if (count <= 0) return
        val remaining = bufferLength - count
        if (remaining > 0) {
            System.arraycopy(buffer, count, buffer, 0, remaining)
        }
        bufferLength = remaining.coerceAtLeast(0)
    }

    private fun parseAa55Packet(pkt: ByteArray, offset: Int, pointCount: Int) {
        val packetTimeNs = SystemClock.elapsedRealtimeNanos()
        val fsaRaw = u8(pkt[offset + 4]) or (u8(pkt[offset + 5]) shl 8)
        val lsaRaw = u8(pkt[offset + 6]) or (u8(pkt[offset + 7]) shl 8)

        val startDeg = ((fsaRaw shr 1) / 64.0f).normalizeAngle()
        var endDeg = ((lsaRaw shr 1) / 64.0f).normalizeAngle()
        val crossedZero = endDeg < startDeg
        if (crossedZero) {
            endDeg += 360.0f
        }

        val angleStep = if (pointCount > 1) (endDeg - startDeg) / (pointCount - 1) else 0f
        for (i in 0 until pointCount) {
            val pointOffset = offset + AA55_HEADER_LENGTH + i * BYTES_PER_POINT
            val low = u8(pkt[pointOffset + 1])
            val high = u8(pkt[pointOffset + 2])
            val distanceMm = (high shl 6) + (low shr 2)
            val angleDeg = (startDeg + angleStep * i).normalizeAngle()
            val previousAngle = lastRawAngleDeg
            if (previousAngle != null && previousAngle - angleDeg > WRAP_DETECTION_DEG) {
                if (waitingForScanBoundary) {
                    clearCurrentScan()
                    waitingForScanBoundary = false
                } else {
                    sendCurrentScan(
                        packetTimeNs,
                        publishAsLiveScan = liveScanAssemblyStartedAtBoundary
                    )
                }
                liveScanAssemblyStartedAtBoundary = true
            }
            addPoint(distanceMm, angleDeg, packetTimeNs)
            lastRawAngleDeg = angleDeg
        }

        maybeSendTimedOutScan(packetTimeNs, crossedZero)
    }

    private fun addPoint(distanceMm: Int, angleDeg: Float, timestampNs: Long) {
        val distanceM = distanceMm / 1000.0f
        // On this device the D6 scan angles already use Cartographer's
        // counter-clockwise convention. Negating them here mirrors every
        // downstream result left-to-right.
        val cartographerAngleDeg = angleDeg.normalizeAngle()
        if (scanSampleAngles.isEmpty()) {
            scanStartNs = timestampNs
        }
        scanSampleAngles.add(cartographerAngleDeg)
        if (distanceM !in MIN_RANGE_M..MAX_RANGE_M) return

        scanRanges.add(distanceM)
        scanAngles.add(cartographerAngleDeg)
        lastShowDistance = distanceM
        lastShowAngle = angleDeg
    }

    private fun maybeSendTimedOutScan(nowNs: Long, crossedZero: Boolean) {
        if (crossedZero || scanStartNs == 0L) return
        if (nowNs - scanStartNs >= MAX_SCAN_ASSEMBLY_NS) {
            // A timeout means the revolution boundary or a packet sector was
            // lost. Feeding this partial fan to Cartographer produces exactly
            // the long radial free-space streaks seen in the self-collected
            // maps. Drop it and wait for the next real zero-degree boundary.
            droppedIncompleteScanCount++
            clearCurrentScan()
            waitingForScanBoundary = true
            liveScanAssemblyStartedAtBoundary = false
        }
    }

    private fun sendCurrentScan(endTimeNs: Long, publishAsLiveScan: Boolean) {
        if (!allowSendLidar) {
            clearCurrentScan()
            return
        }

        val angularCoverage = LidarScanFilter.inspectAngularCoverage(
            scanSampleAngles.toFloatArray()
        )
        lastScanCoverageDeg = angularCoverage.coveredDegrees
        lastScanMaximumGapDeg = angularCoverage.maximumGapDegrees
        if (!angularCoverage.isComplete) {
            droppedIncompleteScanCount++
            clearCurrentScan()
            return
        }
        if (scanRanges.size < MIN_POINTS_PER_SCAN) {
            lastValidPointCount = scanRanges.size
            droppedSparseScanCount++
            clearCurrentScan()
            return
        }

        val observedEndTimeNs = if (endTimeNs > lastTimeNs) {
            endTimeNs
        } else {
            lastTimeNs + 1
        }
        val measuredDurationNs = when {
            lastCompletedScanEndNs > 0L ->
                observedEndTimeNs - lastCompletedScanEndNs
            scanStartNs > 0L -> observedEndTimeNs - scanStartNs
            else -> DEFAULT_SCAN_DURATION_NS
        }
        // Reject scheduling stalls, then slowly track the real motor period.
        // COIN-D6 speed cannot physically jump by several times in one scan.
        val stableMeasurement = measuredDurationNs.coerceIn(
            filteredScanDurationNs / 2,
            filteredScanDurationNs * 3 / 2
        )
        filteredScanDurationNs =
            (filteredScanDurationNs * 7 + stableMeasurement) / 8
        // USB callback time is a delivery timestamp and may jump by several
        // milliseconds as Android schedules the serial thread. Follow the
        // measured motor period and apply only a small phase correction per
        // revolution. Re-anchor after a real pause instead of carrying an old
        // prediction into a new acquisition session.
        lastTimeNs = if (lastCompletedScanEndNs <= 0L ||
            observedEndTimeNs - lastCompletedScanEndNs > TIMESTAMP_REANCHOR_NS) {
            observedEndTimeNs
        } else {
            val predictedEndTimeNs =
                lastCompletedScanEndNs + filteredScanDurationNs
            val phaseErrorNs = (observedEndTimeNs - predictedEndTimeNs)
                .coerceIn(-MAX_TIMESTAMP_PHASE_ERROR_NS, MAX_TIMESTAMP_PHASE_ERROR_NS)
            maxOf(
                lastCompletedScanEndNs + 1L,
                predictedEndTimeNs + phaseErrorNs / TIMESTAMP_PHASE_DIVISOR
            )
        }
        val scanDurationSeconds = filteredScanDurationNs
            .coerceIn(MIN_SCAN_DURATION_NS, MAX_SCAN_DURATION_NS)
            .toFloat() / 1_000_000_000f
        val filteredScan = LidarScanFilter.rejectIsolatedSpikes(
            scanRanges.toFloatArray(),
            scanAngles.toFloatArray()
        )
        rejectedOutlierPointCount += filteredScan.rejectedPointCount.toLong()
        val rangesArray = filteredScan.ranges
        val anglesArray = filteredScan.anglesDeg
        if (rangesArray.size < MIN_POINTS_PER_SCAN) {
            lastValidPointCount = rangesArray.size
            droppedSparseScanCount++
            clearCurrentScan()
            return
        }
        if (publishAsLiveScan) {
            val scan = LidarScan(lastTimeNs, rangesArray, anglesArray)
            synchronized(scanLock) { latestScan = scan }
        }

        lastValidPointCount = rangesArray.size
        acceptedScanCount++
        lastSendNs = lastTimeNs
        lastCompletedScanEndNs = lastTimeNs
        clearCurrentScan()

        carto.addLidarData(lastTimeNs, scanDurationSeconds, rangesArray, anglesArray)
        // Avoid formatting and writing a log entry on every revolution. USB
        // parsing and Cartographer share limited mobile CPU time, so a 1 Hz
        // diagnostic is enough and keeps the acquisition hot path responsive.
        if (lastTimeNs - lastLogNs >= LOG_INTERVAL_NS) {
            lastLogNs = lastTimeNs
            Log.i(
                "LidarFinal",
                "$activeProtocol 发送一帧，点数：${rangesArray.size} " +
                    "周期：${"%.1f".format(scanDurationSeconds * 1000f)}ms " +
                    "AA55:$aa55PacketCount 异常包:$invalidAa55PacketCount " +
                    "残帧:$droppedIncompleteScanCount " +
                    "稀疏帧:$droppedSparseScanCount " +
                    "丢弃字节:$droppedByteCount " +
                    "离群点:$rejectedOutlierPointCount " +
                    "覆盖:${"%.1f".format(lastScanCoverageDeg)}° " +
                    "最大缺口:${"%.1f".format(lastScanMaximumGapDeg)}°"
            )
        }
    }

    private fun clearCurrentScan() {
        scanRanges.clear()
        scanAngles.clear()
        scanSampleAngles.clear()
        scanStartNs = 0L
    }

    private fun Float.normalizeAngle(): Float {
        var angle = this % 360.0f
        if (angle < 0.0f) angle += 360.0f
        return angle
    }

    private fun forwardAngleDelta(fromDeg: Float, toDeg: Float): Float {
        var delta = (toDeg - fromDeg) % 360.0f
        if (delta < 0.0f) delta += 360.0f
        return delta
    }

    private fun u8(value: Byte): Int = value.toInt() and 0xFF

    companion object {
        private const val MIN_HEADER_LENGTH = 4
        private const val AA55_HEADER_LENGTH = 10
        private const val BYTES_PER_POINT = 3
        private const val AA55_MAX_POINTS_PER_PACKET = 200
        private const val AA55_ANGLE_CHECK_BIT = 0x01
        private const val MIN_PACKET_SPAN_DEG = 0.02f
        private const val MAX_PACKET_SPAN_DEG = 90.0f
        private const val MIN_POINT_ANGLE_STEP_DEG = 0.01f
        private const val MAX_POINT_ANGLE_STEP_DEG = 3.0f
        private const val MIN_POINTS_PER_SCAN = 120
        private const val WRAP_DETECTION_DEG = 180f
        private const val DEFAULT_SCAN_DURATION_NS = 100 * 1000_000L
        private const val MIN_SCAN_DURATION_NS = 40 * 1000_000L
        private const val MAX_SCAN_DURATION_NS = 400 * 1000_000L
        private const val MAX_SCAN_ASSEMBLY_NS = 400 * 1000_000L
        private const val TIMESTAMP_REANCHOR_NS = 800 * 1000_000L
        private const val MAX_TIMESTAMP_PHASE_ERROR_NS = 20 * 1000_000L
        private const val TIMESTAMP_PHASE_DIVISOR = 4L
        private const val LOG_INTERVAL_NS = 1_000 * 1000_000L
        private const val MIN_RANGE_M = 0.10f
        private const val MAX_RANGE_M = 8.0f
    }
}

data class LidarScan(
    val timestampNs: Long,
    val ranges: FloatArray,
    val anglesDeg: FloatArray
)
