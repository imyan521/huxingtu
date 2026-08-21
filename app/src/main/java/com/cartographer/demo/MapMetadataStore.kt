package com.cartographer.demo

import org.json.JSONObject
import java.io.File

data class MapNorthAlignment(
    val magneticNorthYawRadians: Float,
    val headingAccuracy: Int,
    val sampleTimestampNs: Long,
    val savedAtMillis: Long = System.currentTimeMillis()
)

object MapMetadataStore {
    fun save(mapFile: File, alignment: MapNorthAlignment): Boolean = try {
        val metadataFile = metadataFileFor(mapFile)
        val temporaryFile = File(metadataFile.parentFile, "${metadataFile.name}.tmp")
        val json = JSONObject().apply {
            put("mapFile", mapFile.name)
            put("magneticNorthYawRadians", alignment.magneticNorthYawRadians.toDouble())
            put("headingAccuracy", alignment.headingAccuracy)
            put("sampleTimestampNs", alignment.sampleTimestampNs)
            put("savedAtMillis", alignment.savedAtMillis)
        }
        temporaryFile.writeText(json.toString(2))
        if (!temporaryFile.renameTo(metadataFile)) {
            temporaryFile.copyTo(metadataFile, overwrite = true)
            temporaryFile.delete()
        }
        metadataFile.exists()
    } catch (_: Exception) {
        false
    }

    fun load(mapFile: File): MapNorthAlignment? {
        return try {
            val metadataFile = metadataFileFor(mapFile)
            if (!metadataFile.exists()) return null
            val json = JSONObject(metadataFile.readText())
            val yaw = json.getDouble("magneticNorthYawRadians").toFloat()
            if (!yaw.isFinite()) return null
            MapNorthAlignment(
                magneticNorthYawRadians = yaw,
                headingAccuracy = json.optInt("headingAccuracy", 0),
                sampleTimestampNs = json.optLong("sampleTimestampNs", 0L),
                savedAtMillis = json.optLong("savedAtMillis", metadataFile.lastModified())
            )
        } catch (_: Exception) {
            null
        }
    }

    fun metadataFileFor(mapFile: File): File = File(
        mapFile.parentFile,
        "${mapFile.nameWithoutExtension}.metadata.json"
    )
}
