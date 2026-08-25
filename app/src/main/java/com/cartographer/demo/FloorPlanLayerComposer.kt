package com.cartographer.demo

import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color

data class FloorPlanLayers(
    val width: Int,
    val height: Int,
    val pointCloud: Bitmap? = null,
    val heatMap: Bitmap? = null,
    val trajectory: Bitmap? = null,
    val floorPlan: Bitmap? = null
)

data class FloorPlanLayerVisibility(
    val pointCloud: Boolean = true,
    val heatMap: Boolean = true,
    val trajectory: Boolean = true,
    val floorPlan: Boolean = true
)

/** Composites registered layers without modifying their source bitmaps. */
object FloorPlanLayerComposer {
    fun compose(
        layers: FloorPlanLayers,
        visibility: FloorPlanLayerVisibility
    ): Bitmap? {
        if (layers.width <= 0 || layers.height <= 0) return null
        val output = Bitmap.createBitmap(
            layers.width,
            layers.height,
            Bitmap.Config.ARGB_8888
        )
        val canvas = Canvas(output)
        canvas.drawColor(Color.WHITE)

        fun draw(bitmap: Bitmap?, enabled: Boolean) {
            if (enabled && bitmap != null && !bitmap.isRecycled &&
                bitmap.width == layers.width && bitmap.height == layers.height) {
                canvas.drawBitmap(bitmap, 0f, 0f, null)
            }
        }

        draw(layers.pointCloud, visibility.pointCloud)
        draw(layers.heatMap, visibility.heatMap)
        draw(layers.trajectory, visibility.trajectory)
        draw(layers.floorPlan, visibility.floorPlan)
        return output
    }
}
