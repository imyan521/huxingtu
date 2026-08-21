package com.cartographer.demo

import android.graphics.PointF

class CartographerNative {

    private var nativeHandle: Long = 0
    private var currentTrajectoryId: Int = -1
    private var lastFinishedTrajectoryId: Int = -1

    init {
        System.loadLibrary("cartographer-jni")
    }

    @Synchronized
    fun initialize(configDirectory: String, configBasename: String): Boolean {
        if (nativeHandle != 0L) return true
        nativeHandle = nativeInit(configDirectory, configBasename)
        return nativeHandle != 0L
    }

    // 🔥 修复：只允许创建一次轨迹，不会重复
    @Synchronized
    fun startTrajectory(): Int {
        if (currentTrajectoryId >= 0) return currentTrajectoryId
        currentTrajectoryId = nativeStartTrajectory(nativeHandle)
        return currentTrajectoryId
    }

    fun addImuData(ts: Long, acc: FloatArray, gyro: FloatArray) {
        if (currentTrajectoryId < 0) return
        nativeAddImuData(
            nativeHandle, currentTrajectoryId, ts,
            acc[0], acc[1], acc[2],
            gyro[0], gyro[1], gyro[2]
        )
    }

    // 🌟 新增：往 C++ 喂雷达数据的入口
    fun addLidarData(
        timestampNs: Long,
        scanDurationSeconds: Float,
        ranges: FloatArray,
        angles: FloatArray
    ) {
        if (currentTrajectoryId < 0 || nativeHandle == 0L) return
        nativeAddRangefinderData(
            nativeHandle,
            currentTrajectoryId,
            timestampNs,
            scanDurationSeconds,
            ranges,
            angles
        )
    }

    fun getCurrentPose(): Pose3D? {
        if (currentTrajectoryId < 0 || nativeHandle == 0L) return null
        val arr = nativeGetPose(nativeHandle, currentTrajectoryId)
        return if (arr.size == 7) Pose3D.fromArray(arr) else null
    }

    fun getLastFinishedPose(): Pose3D? {
        if (lastFinishedTrajectoryId < 0 || nativeHandle == 0L) return null
        val arr = nativeGetPose(nativeHandle, lastFinishedTrajectoryId)
        return if (arr.size == 7) Pose3D.fromArray(arr) else null
    }

    fun getStatus(): SlamStatus {
        if (nativeHandle == 0L) return SlamStatus()
        val arr = nativeGetStatus(nativeHandle)
        return if (arr.size >= 5) {
            SlamStatus(
                rangeFrames = arr[0],
                imuSamples = arr[1],
                localSlamResults = arr[2],
                insertedNodes = arr[3],
                hasPose = arr[4] != 0L
            )
        } else {
            SlamStatus()
        }
    }

    fun getRelocalizationStatus(): RelocalizationStatus {
        if (nativeHandle == 0L) return RelocalizationStatus()
        val arr = nativeGetRelocalizationStatus(nativeHandle)
        return if (arr.size >= 7) {
            RelocalizationStatus(
                oldTrajectoryCount = arr[0].toInt(),
                activeTrajectoryId = arr[1].toInt(),
                activeNodeCount = arr[2].toInt(),
                interTrajectoryConstraintCount = arr[3].toInt(),
                matchedActiveNodeCount = arr[4].toInt(),
                matchedOldSubmapCount = arr[5].toInt(),
                isRelocalized = arr[6] != 0L
            )
        } else {
            RelocalizationStatus()
        }
    }

    fun getLatestSubmapTexture(): SubmapTexture? {
        if (nativeHandle == 0L) return null
        val arr = nativeGetLatestSubmapTexture(nativeHandle)
        return SubmapTexture.fromIntArray(arr)
    }

    fun getActiveSubmapTextures(): List<SubmapTexture> {
        if (nativeHandle == 0L) return emptyList()
        return nativeGetActiveSubmapTextures(nativeHandle)
            .mapNotNull { SubmapTexture.fromIntArray(it) }
    }

    fun getSubmapTextures(): List<SubmapTexture> {
        if (nativeHandle == 0L) return emptyList()
        return nativeGetSubmapTextures(nativeHandle).mapNotNull { SubmapTexture.fromIntArray(it) }
    }

    fun getTrajectoryNodePoses(): List<PointF> {
        if (nativeHandle == 0L) return emptyList()
        val arr = nativeGetTrajectoryNodePoses(nativeHandle)
        if (arr.size < 2) return emptyList()
        val out = ArrayList<PointF>(arr.size / 2)
        var i = 0
        while (i + 1 < arr.size) {
            out.add(PointF(arr[i].toFloat(), arr[i + 1].toFloat()))
            i += 2
        }
        return out
    }

    @Synchronized
    fun finishTrajectory() {
        if (nativeHandle == 0L || currentTrajectoryId < 0) return
        val trajectoryId = currentTrajectoryId
        nativeFinishTrajectory(nativeHandle, trajectoryId)
        lastFinishedTrajectoryId = trajectoryId
        currentTrajectoryId = -1
    }

    fun saveMap(path: String, includeUnfinishedSubmaps: Boolean = false): Boolean {
        if (nativeHandle == 0L) return false
        return nativeSerializeState(nativeHandle, path, includeUnfinishedSubmaps)
    }

    @Synchronized
    fun loadMap(path: String, loadFrozenState: Boolean = true): Boolean {
        if (nativeHandle == 0L || currentTrajectoryId >= 0) return false
        return nativeLoadMap(nativeHandle, path, loadFrozenState)
    }

    @Synchronized
    fun reset() {
        if (nativeHandle != 0L) {
            nativeDestroy(nativeHandle)
        }
        nativeHandle = 0L
        currentTrajectoryId = -1
        lastFinishedTrajectoryId = -1
    }

    private external fun nativeInit(dir: String, file: String): Long
    private external fun nativeStartTrajectory(handle: Long): Int
    private external fun nativeAddImuData(
        handle: Long, trajId: Int, ts: Long,
        ax: Float, ay: Float, az: Float,
        gx: Float, gy: Float, gz: Float
    )

    // 🌟 新增：雷达数据的 JNI 接口
    private external fun nativeAddRangefinderData(
        handle: Long, trajId: Int, timestampNs: Long,
        scanDurationSeconds: Float,
        ranges: FloatArray, angles: FloatArray
    )

    private external fun nativeGetPose(handle: Long, trajId: Int): DoubleArray
    private external fun nativeGetStatus(handle: Long): LongArray
    private external fun nativeGetRelocalizationStatus(handle: Long): LongArray
    private external fun nativeGetLatestSubmapTexture(handle: Long): IntArray
    private external fun nativeGetActiveSubmapTextures(handle: Long): Array<IntArray>
    private external fun nativeGetSubmapTextures(handle: Long): Array<IntArray>
    private external fun nativeGetTrajectoryNodePoses(handle: Long): DoubleArray
    private external fun nativeSerializeState(
        handle: Long,
        path: String,
        includeUnfinishedSubmaps: Boolean
    ): Boolean
    private external fun nativeLoadMap(
        handle: Long,
        path: String,
        loadFrozenState: Boolean
    ): Boolean
    private external fun nativeFinishTrajectory(handle: Long, trajId: Int)
    private external fun nativeDestroy(handle: Long)
}

data class SlamStatus(
    val rangeFrames: Long = 0,
    val imuSamples: Long = 0,
    val localSlamResults: Long = 0,
    val insertedNodes: Long = 0,
    val hasPose: Boolean = false
)

data class RelocalizationStatus(
    val oldTrajectoryCount: Int = 0,
    val activeTrajectoryId: Int = -1,
    val activeNodeCount: Int = 0,
    val interTrajectoryConstraintCount: Int = 0,
    val matchedActiveNodeCount: Int = 0,
    val matchedOldSubmapCount: Int = 0,
    val isRelocalized: Boolean = false
)

data class SubmapTexture(
    val width: Int,
    val height: Int,
    val version: Int,
    val trajectoryId: Int,
    val submapIndex: Int,
    val resolution: Float,
    val originX: Float,
    val originY: Float,
    val theta: Float,
    val pixels: IntArray,
    // Packed original Cartographer texture bytes: intensity << 8 | alpha.
    // Kept separate from the highlighted ARGB pixels used by the live map.
    val rawCells: IntArray = IntArray(0)
) {
    val key: String get() = "$trajectoryId:$submapIndex"

    companion object {
        private const val HEADER_SIZE = 9

        fun fromIntArray(arr: IntArray): SubmapTexture? {
            if (arr.size < HEADER_SIZE) return null
            val width = arr[0]
            val height = arr[1]
            val pixelCount = width * height
            if (width <= 0 || height <= 0 || arr.size < pixelCount + HEADER_SIZE) return null
            return SubmapTexture(
                width = width,
                height = height,
                version = arr[2],
                trajectoryId = arr[3],
                submapIndex = arr[4],
                resolution = Float.fromBits(arr[5]),
                originX = Float.fromBits(arr[6]),
                originY = Float.fromBits(arr[7]),
                theta = Float.fromBits(arr[8]),
                pixels = arr.copyOfRange(HEADER_SIZE, HEADER_SIZE + pixelCount),
                rawCells = if (arr.size >= HEADER_SIZE + pixelCount * 2) {
                    arr.copyOfRange(
                        HEADER_SIZE + pixelCount,
                        HEADER_SIZE + pixelCount * 2
                    )
                } else {
                    IntArray(0)
                }
            )
        }
    }
}

data class Pose3D(val position: Triple<Double, Double, Double>, val orientation: Quaternion) {
    companion object {
        fun fromArray(array: DoubleArray): Pose3D {
            return Pose3D(
                Triple(array[0], array[1], array[2]),
                Quaternion(array[3], array[4], array[5], array[6])
            )
        }
    }
    fun to2D() = Pose2D(position.first, position.second, orientation.toEulerAngles().third)
}

data class Pose2D(val x: Double, val y: Double, val theta: Double)
data class Quaternion(val w: Double, val x: Double, val y: Double, val z: Double) {
    fun toEulerAngles(): Triple<Double, Double, Double> {
        val yaw = kotlin.math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return Triple(0.0, 0.0, yaw)
    }
}
