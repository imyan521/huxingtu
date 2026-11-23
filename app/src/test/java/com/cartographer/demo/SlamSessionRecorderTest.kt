package com.cartographer.demo

import java.io.DataInputStream
import java.io.File
import org.junit.Assert.*
import org.junit.Rule
import org.junit.Test
import org.junit.rules.TemporaryFolder

class SlamSessionRecorderTest {
    @get:Rule val temp = TemporaryFolder()

    @Test fun recordsOwnCopiesAndDrainsBeforeClosing() {
        val config = temp.newFolder("config")
        File(config, "android_2d.lua").writeText("original")
        val directory = File(temp.root, "session")
        val recorder = SlamSessionRecorder(directory, config)
        val bytes = byteArrayOf(1, 2, 3)
        recorder.usb(100L, bytes)
        bytes[0] = 99
        recorder.imu(200L, floatArrayOf(1f, 2f, 3f), floatArrayOf(4f, 5f, 6f))
        recorder.scan(300L, .1f, floatArrayOf(2f), floatArrayOf(45f))
        recorder.pose(400L, doubleArrayOf(1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0))
        File(config, "android_2d.lua").writeText("changed")
        recorder.stop()
        assertTrue(recorder.awaitClosed(5000))
        assertNull(recorder.failure)
        assertEquals("original", File(directory, "android_2d.lua").readText())
        DataInputStream(File(directory, "sensors.bin").inputStream()).use { input ->
            val magic = ByteArray(8); input.readFully(magic)
            assertEquals("CSLAM001", String(magic))
            assertEquals(12, input.readInt()); assertEquals(1, input.readUnsignedByte())
            assertEquals(100L, input.readLong())
            val raw = ByteArray(3); input.readFully(raw)
            assertArrayEquals(byteArrayOf(1, 2, 3), raw)
            assertEquals(33, input.readInt()); assertEquals(2, input.readUnsignedByte())
            assertEquals(200L, input.readLong())
            for (i in 1..6) assertEquals(i.toFloat(), input.readFloat(), 0f)
            assertEquals(25, input.readInt()); assertEquals(3, input.readUnsignedByte())
            assertEquals(300L, input.readLong()); assertEquals(.1f, input.readFloat(), 0f)
            assertEquals(1, input.readInt()); assertEquals(2f, input.readFloat(), 0f)
            assertEquals(45f, input.readFloat(), 0f)
            assertEquals(65, input.readInt()); assertEquals(4, input.readUnsignedByte())
            assertEquals(400L, input.readLong())
            for (value in doubleArrayOf(1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0)) {
                assertEquals(value, input.readDouble(), 0.0)
            }
            assertEquals(18, input.readInt()); assertEquals(127, input.readUnsignedByte())
            assertEquals(0L, input.readLong()); assertEquals(0L, input.readLong())
            assertFalse(input.readBoolean()); assertEquals(-1, input.read())
        }
    }

    @Test fun sizeCapIsExplicitAndDoesNotBlockProducers() {
        val config = temp.newFolder("config")
        val directory = File(temp.root, "capped")
        val recorder = SlamSessionRecorder(directory, config, maxBytes = 80)
        recorder.usb(100L, ByteArray(100))
        assertTrue(recorder.awaitClosed(5000))
        DataInputStream(File(directory, "sensors.bin").inputStream()).use {
            it.skipBytes(8); assertEquals(18, it.readInt()); assertEquals(127, it.readUnsignedByte())
            it.readLong(); assertTrue(it.readLong() >= 1); assertTrue(it.readBoolean())
        }
    }

    @Test fun diskFailureClosesRecordingWithoutThrowingIntoAcquisition() {
        val invalidDirectory = temp.newFile("not_a_directory")
        val recorder = SlamSessionRecorder(invalidDirectory, temp.newFolder("config"))
        recorder.usb(100L, ByteArray(10))
        assertTrue(recorder.awaitClosed(5000)); assertNotNull(recorder.failure)
        recorder.usb(200L, ByteArray(10))
        recorder.stop()
    }
}
