package com.cartographer.demo

import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.RectF
import java.util.Locale
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.min

/** Adds dimensions to a transparent floor-plan layer without changing its frame. */
object FloorPlanDimensionOverlayRenderer {
    private const val MIN_EDGE_METERS = 1.5f

    fun render(
        floorPlanOverlay: Bitmap,
        generation: FloorPlanGenerationInfo,
        metersPerPixel: Float
    ): Bitmap? {
        val vertices = generation.outlineVerticesPixels
        if (vertices.size < 3 || !metersPerPixel.isFinite() || metersPerPixel <= 0f) {
            return null
        }
        val output = floorPlanOverlay.copy(Bitmap.Config.ARGB_8888, true)
        val canvas = Canvas(output)
        val shortSide = min(output.width, output.height).toFloat().coerceAtLeast(1f)
        val textSize = (shortSide * 0.022f).coerceIn(10f, 24f)
        val textPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.BLACK
            this.textSize = textSize
            textAlign = Paint.Align.CENTER
        }
        val backgroundPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.argb(225, 255, 255, 255)
            style = Paint.Style.FILL
        }

        var signedAreaTwice = 0f
        vertices.forEachIndexed { index, point ->
            val next = vertices[(index + 1) % vertices.size]
            signedAreaTwice += point.x * next.y - next.x * point.y
        }
        val outwardSign = if (signedAreaTwice >= 0f) 1f else -1f
        val offset = textSize * 0.85f
        vertices.forEachIndexed { index, start ->
            val end = vertices[(index + 1) % vertices.size]
            val dx = end.x - start.x
            val dy = end.y - start.y
            val pixelLength = hypot(dx, dy)
            val meters = pixelLength * metersPerPixel
            if (!meters.isFinite() || meters < MIN_EDGE_METERS) return@forEachIndexed
            val normalX = outwardSign * dy / pixelLength
            val normalY = outwardSign * -dx / pixelLength
            drawLabel(
                canvas,
                (start.x + end.x) * 0.5f + normalX * offset,
                (start.y + end.y) * 0.5f + normalY * offset,
                Math.toDegrees(atan2(dy, dx).toDouble()).toFloat(),
                String.format(Locale.US, "%.2f m", meters),
                textPaint,
                backgroundPaint
            )
        }
        return output
    }

    private fun drawLabel(
        canvas: Canvas,
        centerX: Float,
        centerY: Float,
        sourceAngle: Float,
        label: String,
        textPaint: Paint,
        backgroundPaint: Paint
    ) {
        var angle = sourceAngle
        while (angle > 90f) angle -= 180f
        while (angle <= -90f) angle += 180f
        canvas.save()
        canvas.rotate(angle, centerX, centerY)
        val metrics = textPaint.fontMetrics
        val baseline = centerY - (metrics.ascent + metrics.descent) * 0.5f
        val halfWidth = textPaint.measureText(label) * 0.5f
        val padX = textPaint.textSize * 0.22f
        val padY = textPaint.textSize * 0.10f
        canvas.drawRoundRect(
            RectF(
                centerX - halfWidth - padX,
                baseline + metrics.ascent - padY,
                centerX + halfWidth + padX,
                baseline + metrics.descent + padY
            ),
            padY,
            padY,
            backgroundPaint
        )
        canvas.drawText(label, centerX, baseline, textPaint)
        canvas.restore()
    }
}
