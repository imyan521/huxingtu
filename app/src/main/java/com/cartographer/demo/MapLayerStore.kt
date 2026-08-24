package com.cartographer.demo

import org.json.JSONArray
import org.json.JSONObject
import java.io.File

/** Persistent sidecars that stay registered to one pbstream map. */
object MapLayerStore {
    fun rssiFileFor(mapFile: File) = sidecar(mapFile, "rssi.json")
    fun pointCloudFileFor(mapFile: File) = sidecar(mapFile, "pointcloud.png")
    fun floorPlanFileFor(mapFile: File) = sidecar(mapFile, "floorplan_overlay.png")
    fun heatMapFileFor(mapFile: File) = sidecar(mapFile, "heatmap_overlay.png")
    fun trajectoryFileFor(mapFile: File) = sidecar(mapFile, "trajectory_overlay.png")

    fun saveRssiSamples(mapFile: File, samples: List<RssiSample>): Boolean = try {
        val array = JSONArray()
        samples.forEach { sample ->
            array.put(JSONObject().apply {
                put("worldX", sample.worldX.toDouble())
                put("worldY", sample.worldY.toDouble())
                put("rssiDbm", sample.rssiDbm.toDouble())
                put("timestampMillis", sample.timestampMillis)
            })
        }
        writeTextAtomically(
            rssiFileFor(mapFile),
            JSONObject().apply {
                put("mapFile", mapFile.name)
                put("coordinateFrame", "cartographer_world")
                put("samples", array)
            }.toString(2)
        )
    } catch (_: Exception) {
        false
    }

    fun loadRssiSamples(mapFile: File): List<RssiSample> {
        val file = rssiFileFor(mapFile)
        if (!file.exists()) return emptyList()
        return try {
            val array = JSONObject(file.readText()).getJSONArray("samples")
            buildList(array.length()) {
                for (index in 0 until array.length()) {
                    val item = array.getJSONObject(index)
                    val sample = RssiSample(
                        worldX = item.getDouble("worldX").toFloat(),
                        worldY = item.getDouble("worldY").toFloat(),
                        rssiDbm = item.getDouble("rssiDbm").toFloat(),
                        timestampMillis = item.optLong("timestampMillis", 0L)
                    )
                    if (sample.worldX.isFinite() && sample.worldY.isFinite() &&
                        sample.rssiDbm.isFinite()) add(sample)
                }
            }
        } catch (_: Exception) {
            emptyList()
        }
    }

    fun saveRenderedLayers(
        mapFile: File,
        pointCloud: File?,
        floorPlanOverlay: File?,
        heatMapOverlay: File?,
        trajectoryOverlay: File?
    ): Boolean = try {
        pointCloud?.takeIf { it.exists() }?.copyTo(
            pointCloudFileFor(mapFile), overwrite = true
        )
        floorPlanOverlay?.takeIf { it.exists() }?.copyTo(
            floorPlanFileFor(mapFile), overwrite = true
        )
        heatMapOverlay?.takeIf { it.exists() }?.copyTo(
            heatMapFileFor(mapFile), overwrite = true
        )
        trajectoryOverlay?.takeIf { it.exists() }?.copyTo(
            trajectoryFileFor(mapFile), overwrite = true
        )
        true
    } catch (_: Exception) {
        false
    }

    private fun sidecar(mapFile: File, suffix: String) = File(
        mapFile.parentFile,
        "${mapFile.nameWithoutExtension}.$suffix"
    )

    private fun writeTextAtomically(file: File, text: String): Boolean {
        val temporary = File(file.parentFile, "${file.name}.tmp")
        temporary.writeText(text)
        if (!temporary.renameTo(file)) {
            temporary.copyTo(file, overwrite = true)
            temporary.delete()
        }
        return file.exists() && file.length() > 0L
    }
}
