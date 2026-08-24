package com.cartographer.demo

import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import kotlin.math.min

object TrajectoryOverlayRenderer {
    fun render(
        width: Int,
        height: Int,
        points: List<FloorPlanPixelPoint>
    ): Bitmap? {
        if (width <= 0 || height <= 0 || points.size < 2) return null
        val output = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888)
        val path = Path()
        points.forEachIndexed { index, point ->
            if (index == 0) path.moveTo(point.x, point.y)
            else path.lineTo(point.x, point.y)
        }
        val paint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.argb(235, 255, 170, 25)
            style = Paint.Style.STROKE
            strokeWidth = (min(width, height) * 0.005f).coerceIn(2f, 6f)
            strokeCap = Paint.Cap.ROUND
            strokeJoin = Paint.Join.ROUND
        }
        Canvas(output).drawPath(path, paint)
        return output
    }
}
