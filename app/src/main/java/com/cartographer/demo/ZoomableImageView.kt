package com.cartographer.demo

import android.content.Context
import android.graphics.Bitmap
import android.graphics.Matrix
import android.graphics.RectF
import android.util.AttributeSet
import android.view.MotionEvent
import android.view.ScaleGestureDetector
import androidx.appcompat.widget.AppCompatImageView
import kotlin.math.min

class ZoomableImageView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null
) : AppCompatImageView(context, attrs) {

    private val contentMatrix = Matrix()
    private val imageBounds = RectF()
    private var fitScale = 1f
    private var currentScale = 1f
    private var lastX = 0f
    private var lastY = 0f
    private var dragging = false

    private val scaleDetector = ScaleGestureDetector(
        context,
        object : ScaleGestureDetector.SimpleOnScaleGestureListener() {
            override fun onScaleBegin(detector: ScaleGestureDetector): Boolean {
                parent?.requestDisallowInterceptTouchEvent(true)
                return true
            }

            override fun onScale(detector: ScaleGestureDetector): Boolean {
                val maxScale = fitScale * MAX_ZOOM
                val desired = (currentScale * detector.scaleFactor).coerceIn(fitScale, maxScale)
                val factor = desired / currentScale
                if (factor.isFinite() && factor > 0f) {
                    contentMatrix.postScale(factor, factor, detector.focusX, detector.focusY)
                    currentScale = desired
                    constrainToView()
                    imageMatrix = contentMatrix
                }
                return true
            }
        }
    )

    init {
        scaleType = ScaleType.MATRIX
        setBackgroundColor(0xFFFFFFFF.toInt())
    }

    override fun setImageBitmap(bm: Bitmap?) {
        super.setImageBitmap(bm)
        post { resetZoom() }
    }

    override fun onSizeChanged(w: Int, h: Int, oldw: Int, oldh: Int) {
        super.onSizeChanged(w, h, oldw, oldh)
        resetZoom()
    }

    override fun onTouchEvent(event: MotionEvent): Boolean {
        scaleDetector.onTouchEvent(event)
        when (event.actionMasked) {
            MotionEvent.ACTION_DOWN -> {
                parent?.requestDisallowInterceptTouchEvent(true)
                lastX = event.x
                lastY = event.y
                dragging = true
            }
            MotionEvent.ACTION_POINTER_DOWN -> dragging = false
            MotionEvent.ACTION_MOVE -> {
                if (dragging && event.pointerCount == 1 && !scaleDetector.isInProgress) {
                    contentMatrix.postTranslate(event.x - lastX, event.y - lastY)
                    lastX = event.x
                    lastY = event.y
                    constrainToView()
                    imageMatrix = contentMatrix
                }
            }
            MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                dragging = false
                parent?.requestDisallowInterceptTouchEvent(false)
            }
        }
        return true
    }

    fun resetZoom() {
        val drawable = drawable ?: return
        if (width <= 0 || height <= 0 || drawable.intrinsicWidth <= 0 || drawable.intrinsicHeight <= 0) return

        val availableWidth = width - paddingLeft - paddingRight
        val availableHeight = height - paddingTop - paddingBottom
        fitScale = min(
            availableWidth / drawable.intrinsicWidth.toFloat(),
            availableHeight / drawable.intrinsicHeight.toFloat()
        ).coerceAtLeast(0.001f)
        currentScale = fitScale

        val dx = paddingLeft + (availableWidth - drawable.intrinsicWidth * fitScale) * 0.5f
        val dy = paddingTop + (availableHeight - drawable.intrinsicHeight * fitScale) * 0.5f
        contentMatrix.reset()
        contentMatrix.postScale(fitScale, fitScale)
        contentMatrix.postTranslate(dx, dy)
        imageMatrix = contentMatrix
    }

    private fun constrainToView() {
        val drawable = drawable ?: return
        imageBounds.set(0f, 0f, drawable.intrinsicWidth.toFloat(), drawable.intrinsicHeight.toFloat())
        contentMatrix.mapRect(imageBounds)

        val leftLimit = paddingLeft.toFloat()
        val topLimit = paddingTop.toFloat()
        val rightLimit = (width - paddingRight).toFloat()
        val bottomLimit = (height - paddingBottom).toFloat()

        val dx = when {
            imageBounds.width() <= rightLimit - leftLimit -> (leftLimit + rightLimit) * 0.5f - imageBounds.centerX()
            imageBounds.left > leftLimit -> leftLimit - imageBounds.left
            imageBounds.right < rightLimit -> rightLimit - imageBounds.right
            else -> 0f
        }
        val dy = when {
            imageBounds.height() <= bottomLimit - topLimit -> (topLimit + bottomLimit) * 0.5f - imageBounds.centerY()
            imageBounds.top > topLimit -> topLimit - imageBounds.top
            imageBounds.bottom < bottomLimit -> bottomLimit - imageBounds.bottom
            else -> 0f
        }
        contentMatrix.postTranslate(dx, dy)
    }

    companion object {
        private const val MAX_ZOOM = 8f
    }
}
