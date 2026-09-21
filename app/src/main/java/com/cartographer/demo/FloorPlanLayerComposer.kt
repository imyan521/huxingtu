package com.cartographer.demo

import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Path

data class FloorPlanLayers(
    val width: Int,
    val height: Int,
    val pointCloud: Bitmap? = null,
    val heatMap: Bitmap? = null,
    val trajectory: Bitmap? = null,
    val floorPlan: Bitmap? = null,
    val pointCloudOutline: List<FloorPlanPixelPoint>? = null
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

        val outline = layers.pointCloudOutline?.takeIf { vertices ->
            vertices.size >= 3 && vertices.all { it.x.isFinite() && it.y.isFinite() }
        }
        if (visibility.pointCloud && outline != null) {
            val clipPath = Path().apply {
                moveTo(outline[0].x, outline[0].y)
                for (index in 1 until outline.size) {
                    lineTo(outline[index].x, outline[index].y)
                }
                close()
            }
            val checkpoint = canvas.save()
            canvas.clipPath(clipPath)
            draw(layers.pointCloud, true)
            canvas.restoreToCount(checkpoint)
        } else {
            draw(layers.pointCloud, visibility.pointCloud)
        }
        draw(layers.heatMap, visibility.heatMap)
        draw(layers.trajectory, visibility.trajectory)
        draw(layers.floorPlan, visibility.floorPlan)
        return output
    }
}
