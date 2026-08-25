package com.cartographer.demo

import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import kotlin.math.hypot
import kotlin.math.min

/** Renders the optimized trajectory as a transparent floor-plan layer. */
object TrajectoryOverlayRenderer {
    fun render(
        width: Int,
        height: Int,
        points: List<FloorPlanPixelPoint>
    ): Bitmap? {
        if (width <= 0 || height <= 0 || points.size < 2) return null
        val filtered = ArrayList<FloorPlanPixelPoint>(points.size)
        for (point in points) {
            if (!point.x.isFinite() || !point.y.isFinite()) continue
            val previous = filtered.lastOrNull()
            if (previous == null || hypot(point.x - previous.x, point.y - previous.y) >= 0.5f) {
                filtered += point
            }
        }
        if (filtered.size < 2) return null

        val output = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888)
        val canvas = Canvas(output)
        val path = Path()
        filtered.forEachIndexed { index, point ->
            if (index == 0) path.moveTo(point.x, point.y) else path.lineTo(point.x, point.y)
        }
        val shortSide = min(width, height).toFloat()
        val strokeWidth = (shortSide * 0.005f).coerceIn(2f, 7f)
        val pathPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.argb(235, 255, 170, 25)
            style = Paint.Style.STROKE
            this.strokeWidth = strokeWidth
            strokeCap = Paint.Cap.ROUND
            strokeJoin = Paint.Join.ROUND
        }
        canvas.drawPath(path, pathPaint)

        val endpointRadius = (strokeWidth * 1.6f).coerceAtLeast(4f)
        val endpointPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            style = Paint.Style.FILL
        }
        endpointPaint.color = Color.rgb(35, 190, 80)
        canvas.drawCircle(filtered.first().x, filtered.first().y, endpointRadius, endpointPaint)
        endpointPaint.color = Color.rgb(225, 55, 45)
        canvas.drawCircle(filtered.last().x, filtered.last().y, endpointRadius, endpointPaint)
        return output
    }
}
