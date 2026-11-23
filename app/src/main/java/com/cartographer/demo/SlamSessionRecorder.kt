package com.cartographer.demo

import java.io.ByteArrayOutputStream
import java.io.DataOutputStream
import java.io.File
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicLong

/** Big-endian framed sensor records; disk I/O never runs on sensor threads. */
internal class SlamSessionRecorder(
    private val directory: File,
    private val configDirectory: File,
    private val maxBytes: Long = 64L * 1024 * 1024,
    queueCapacity: Int = 1024
) {
    private val queue = ArrayBlockingQueue<ByteArray>(queueCapacity)
    private val dropped = AtomicLong()
    private val configuration = configDirectory.listFiles()
        ?.filter { it.extension == "lua" }?.associate { it.name to it.readBytes() }.orEmpty()
    @Volatile private var accepting = true
    @Volatile var failure: String? = null
        private set
    private val writer = Thread({ writeSession() }, "SlamRecording").apply { start() }

    private fun record(type: Int, timestamp: Long, encode: (DataOutputStream) -> Unit) {
        if (!accepting) return
        val bytes = ByteArrayOutputStream()
        DataOutputStream(bytes).use { out ->
            out.writeByte(type)
            out.writeLong(timestamp)
            encode(out)
        }
        synchronized(this) {
            if (accepting && !queue.offer(bytes.toByteArray())) dropped.incrementAndGet()
        }
    }

    fun usb(timestamp: Long, bytes: ByteArray) = record(1, timestamp) { it.write(bytes) }

    fun imu(timestamp: Long, acc: FloatArray, gyro: FloatArray) = record(2, timestamp) {
        out -> for (i in 0..2) out.writeFloat(acc[i])
        for (i in 0..2) out.writeFloat(gyro[i])
    }

    fun scan(timestamp: Long, duration: Float, ranges: FloatArray, angles: FloatArray) =
        record(3, timestamp) { out ->
            out.writeFloat(duration)
            val count = minOf(ranges.size, angles.size)
            out.writeInt(count)
            for (i in 0 until count) { out.writeFloat(ranges[i]); out.writeFloat(angles[i]) }
        }

    fun pose(timestamp: Long, pose: DoubleArray) = record(4, timestamp) { out ->
        for (i in 0..6) out.writeDouble(pose[i])
    }

    @Synchronized fun stop() { accepting = false }

    // For tests and explicit export completion; never called on acquisition/UI threads.
    fun awaitClosed(timeoutMillis: Long): Boolean {
        writer.join(timeoutMillis)
        return !writer.isAlive
    }

    private fun writeSession() {
        try {
            check(directory.mkdirs() || directory.isDirectory)
            configuration.forEach { (name, bytes) ->
                File(directory, name).writeBytes(bytes)
            }
            DataOutputStream(File(directory, "sensors.bin").outputStream().buffered()).use { out ->
                out.writeBytes("CSLAM001")
                var written = 8L
                var capped = false
                while (accepting || queue.isNotEmpty()) {
                    val frame = queue.poll(100L, TimeUnit.MILLISECONDS) ?: continue
                    if (written + 4 + frame.size > maxBytes) {
                        synchronized(this) {
                            accepting = false
                            dropped.addAndGet(1L + queue.size)
                            queue.clear()
                        }
                        capped = true
                        break
                    }
                    out.writeInt(frame.size)
                    out.write(frame)
                    written += 4 + frame.size
                }
                // Footer: type 127, zero timestamp, dropped count, capped flag.
                out.writeInt(18)
                out.writeByte(127)
                out.writeLong(0L)
                out.writeLong(dropped.get())
                out.writeBoolean(capped)
            }
        } catch (e: Exception) {
            failure = e.toString()
        } finally {
            stop()
            queue.clear()
        }
    }
}
