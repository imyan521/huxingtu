package com.cartographer.demo

import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import android.graphics.Rect
import android.graphics.RectF
import kotlin.math.ceil
import kotlin.math.max
import kotlin.math.min

data class RssiSample(
    val worldX: Float,
    val worldY: Float,
    val rssiDbm: Float,
    val timestampMillis: Long = 0L
)

/** Renders RSSI samples in exactly the same pixel frame as floor-plan export. */
object HeatMapRenderer {
    private const val GRID_SIZE = 128

    fun render(
        samples: List<RssiSample>,
        geometry: FloorPlanMapExporter.ExportGeometry,
        outlinePixels: List<FloorPlanPixelPoint>
    ): Bitmap? {
        val validSamples = samples.filter {
            it.worldX.isFinite() && it.worldY.isFinite() &&
                it.rssiDbm.isFinite() && it.rssiDbm in -150f..0f
        }
        if (validSamples.isEmpty() || outlinePixels.size < 3 ||
            geometry.widthPx <= 0 || geometry.heightPx <= 0) return null

        val gridWidth = min(GRID_SIZE, geometry.widthPx)
        val gridHeight = min(
            GRID_SIZE,
            max(1, ceil(gridWidth * geometry.heightPx.toDouble() /
                geometry.widthPx.toDouble()).toInt())
        )
        val pixels = IntArray(gridWidth * gridHeight)
        for (gridY in 0 until gridHeight) {
            val pixelY = (gridY + 0.5f) * geometry.heightPx / gridHeight
            val worldY = geometry.worldMaxY -
                pixelY * geometry.resolutionMetersPerPixel
            for (gridX in 0 until gridWidth) {
                val pixelX = (gridX + 0.5f) * geometry.widthPx / gridWidth
                val worldX = geometry.worldMinX +
                    pixelX * geometry.resolutionMetersPerPixel
                var weightedRssi = 0.0
                var weightSum = 0.0
                for (sample in validSamples) {
                    val dx = worldX - sample.worldX
                    val dy = worldY - sample.worldY
                    val distanceSquared = (dx * dx + dy * dy).toDouble()
                        .coerceAtLeast(0.0025)
                    val weight = 1.0 / distanceSquared
                    weightedRssi += sample.rssiDbm * weight
                    weightSum += weight
                }
                pixels[gridY * gridWidth + gridX] =
                    rssiColor((weightedRssi / weightSum).toFloat())
            }
        }

        val coarse = Bitmap.createBitmap(
            pixels, gridWidth, gridHeight, Bitmap.Config.ARGB_8888
        )
        val output = Bitmap.createBitmap(
            geometry.widthPx, geometry.heightPx, Bitmap.Config.ARGB_8888
        )
        val canvas = Canvas(output)
        val clip = Path().apply {
            outlinePixels.forEachIndexed { index, point ->
                if (index == 0) moveTo(point.x, point.y) else lineTo(point.x, point.y)
            }
            close()
        }
        canvas.save()
        canvas.clipPath(clip)
        canvas.drawBitmap(
            coarse,
            Rect(0, 0, coarse.width, coarse.height),
            RectF(0f, 0f, output.width.toFloat(), output.height.toFloat()),
            Paint(Paint.FILTER_BITMAP_FLAG)
        )
        canvas.restore()
        coarse.recycle()
        return output
    }

    private fun rssiColor(rssi: Float): Int {
        val stops = arrayOf(
            -90f to Color.rgb(35, 80, 210),
            -75f to Color.rgb(30, 190, 210),
            -65f to Color.rgb(45, 205, 85),
            -55f to Color.rgb(245, 215, 35),
            -45f to Color.rgb(245, 125, 25),
            -30f to Color.rgb(225, 45, 35)
        )
        val clamped = rssi.coerceIn(stops.first().first, stops.last().first)
        for (index in 0 until stops.lastIndex) {
            val (lowValue, lowColor) = stops[index]
            val (highValue, highColor) = stops[index + 1]
            if (clamped <= highValue) {
                val fraction = (clamped - lowValue) / (highValue - lowValue)
                fun channel(low: Int, high: Int): Int =
                    (low + (high - low) * fraction).toInt().coerceIn(0, 255)
                return Color.argb(
                    210,
                    channel(Color.red(lowColor), Color.red(highColor)),
                    channel(Color.green(lowColor), Color.green(highColor)),
                    channel(Color.blue(lowColor), Color.blue(highColor))
                )
            }
        }
        return Color.argb(210, 225, 45, 35)
    }
}
