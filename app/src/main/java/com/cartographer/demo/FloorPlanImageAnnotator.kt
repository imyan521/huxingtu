package com.cartographer.demo

import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.RectF
import java.io.File
import java.io.FileOutputStream
import java.util.Locale
import kotlin.math.atan2
import kotlin.math.ceil
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.roundToInt

object FloorPlanImageAnnotator {
    private const val MIN_ANNOTATED_EDGE_METERS = 1.50f

    data class Result(
        val annotated: Boolean,
        val failureReason: String? = null,
        val lengthMeters: Float = 0f,
        val widthMeters: Float = 0f,
        val boundingAreaSquareMeters: Float = 0f
    )

    fun annotateFile(
        file: File,
        generation: FloorPlanGenerationInfo,
        metersPerPixel: Float
    ): Result {
        if (!file.exists()) return Result(false, "户型图结果文件不存在")
        if (!metersPerPixel.isFinite() || metersPerPixel <= 0f) {
            return Result(false, "户型图分辨率无效")
        }

        var lengthMeters = generation.dimensionLongSizePixels * metersPerPixel
        var widthMeters = generation.dimensionShortSizePixels * metersPerPixel
        val area = generation.footprintAreaPixelsSquared *
            metersPerPixel * metersPerPixel
        if (!lengthMeters.isFinite() || !widthMeters.isFinite() ||
            !area.isFinite() || lengthMeters <= 0f || widthMeters <= 0f) {
            return Result(false, "绿色闭合边界尺寸无效")
        }

        var source: Bitmap? = null
        var croppedSource: Bitmap? = null
        var annotated: Bitmap? = null
        return try {
            val decoded = BitmapFactory.decodeFile(file.absolutePath)
                ?: return Result(false, "无法读取户型图", lengthMeters = lengthMeters,
                    widthMeters = widthMeters, boundingAreaSquareMeters = area)
            source = decoded
            val geometry = validatedGeometry(decoded, generation)
                ?: return Result(false, "绿色闭合边界旋转几何无效", lengthMeters = lengthMeters,
                    widthMeters = widthMeters, boundingAreaSquareMeters = area)
            // The annotation must use the exact polygon that was drawn in green,
            // not a stale/native bounding-box size.
            lengthMeters = geometry.longSizePixels * metersPerPixel
            widthMeters = geometry.shortSizePixels * metersPerPixel
            // Geometry extraction must stay in the original SLAM pixel frame so
            // map overlays remain registered.  The report is a presentation
            // frame, however: rotate the complete raster (background, green
            // outline and red partitions together) by the small residual angle
            // to the nearest Manhattan axis before cropping/annotating it.
            // Previously we only cropped here, which exposed the original map
            // yaw and made an otherwise orthogonal polygon look visibly skewed.
            val prepared = canonicalizeAndCrop(
                source = decoded,
                geometry = geometry
            )
            croppedSource = prepared.bitmap
            val annotatedBitmap = drawAnnotations(
                prepared.bitmap,
                prepared.geometry,
                lengthMeters,
                widthMeters,
                area,
                metersPerPixel
            )
            annotated = annotatedBitmap
            if (!writePngSafely(file, annotatedBitmap)) {
                Result(false, "带四边尺寸标注的户型图写入失败",
                    lengthMeters = lengthMeters, widthMeters = widthMeters,
                    boundingAreaSquareMeters = area)
            } else {
                Result(
                    annotated = true,
                    lengthMeters = lengthMeters,
                    widthMeters = widthMeters,
                    boundingAreaSquareMeters = area
                )
            }
        } catch (_: OutOfMemoryError) {
            Result(false, "图片过大，设备内存不足", lengthMeters = lengthMeters,
                widthMeters = widthMeters, boundingAreaSquareMeters = area)
        } catch (e: Exception) {
            Result(false, e.message ?: "尺寸标注发生未知错误",
                lengthMeters = lengthMeters, widthMeters = widthMeters,
                boundingAreaSquareMeters = area)
        } finally {
            annotated?.let { if (it !== source && !it.isRecycled) it.recycle() }
            croppedSource?.let {
                if (it !== source && it !== annotated && !it.isRecycled) it.recycle()
            }
            source?.let { if (!it.isRecycled) it.recycle() }
        }
    }

    /**
     * Last-resort path used only when the native topology reconstruction cannot
     * produce a polygon. Internal walls must not participate in orientation
     * estimation: they can be denser than the outside walls and used to pull the
     * old PCA rectangle away from the actual building. Instead, connect only
     * small raster gaps, keep the largest wall component, take its convex hull,
     * and fit the minimum-area rectangle to that outside evidence.
     */
    fun annotateFallbackFile(file: File, metersPerPixel: Float): Result {
        if (!file.exists() || !metersPerPixel.isFinite() || metersPerPixel <= 0f) {
            return Result(false, "户型图兜底输入无效")
        }
        var source: Bitmap? = null
        var greenBase: Bitmap? = null
        return try {
            val bitmap = BitmapFactory.decodeFile(file.absolutePath)
                ?: return Result(false, "无法读取户型图兜底图")
            source = bitmap
            val step = max(1, max(bitmap.width, bitmap.height) / 1200)
            val gridWidth = (bitmap.width + step - 1) / step
            val gridHeight = (bitmap.height + step - 1) / step
            val darkMask = BooleanArray(gridWidth * gridHeight)
            var darkCount = 0
            for (gridY in 0 until gridHeight) {
                val pixelY = min(bitmap.height - 1, gridY * step)
                for (gridX in 0 until gridWidth) {
                    val pixelX = min(bitmap.width - 1, gridX * step)
                    val color = bitmap.getPixel(pixelX, pixelY)
                    val luminance = (
                        Color.red(color) * 299 +
                            Color.green(color) * 587 +
                            Color.blue(color) * 114
                        ) / 1000
                    if (Color.alpha(color) > 32 && luminance < 190) {
                        darkMask[gridY * gridWidth + gridX] = true
                        darkCount++
                    }
                }
            }
            if (darkCount < 100) {
                return Result(false, "稳定墙体像素不足，无法生成尺寸兜底")
            }
            val wallPoints = largestConnectedWallPoints(
                darkMask,
                gridWidth,
                gridHeight,
                step
            )
            val hull = convexHull(wallPoints)
            val fitted = minimumAreaRectangle(hull)
                ?: return Result(false, "无法从外层墙体拟合闭合边界")
            val vertices = fitted.vertices
            if (vertices.any {
                    !it.x.isFinite() || !it.y.isFinite() ||
                        it.x !in -1f..(bitmap.width + 1f) ||
                        it.y !in -1f..(bitmap.height + 1f)
                }) {
                return Result(false, "稳定墙体边界超出图片")
            }
            val fallbackBitmap = bitmap.copy(Bitmap.Config.ARGB_8888, true)
            greenBase = fallbackBitmap
            val canvas = Canvas(fallbackBitmap)
            val greenPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
                color = Color.rgb(0, 235, 40)
                style = Paint.Style.STROKE
                strokeWidth = (min(bitmap.width, bitmap.height) * 0.006f)
                    .coerceIn(2f, 7f)
                strokeJoin = Paint.Join.ROUND
            }
            vertices.forEachIndexed { index, start ->
                val end = vertices[(index + 1) % vertices.size]
                canvas.drawLine(start.x, start.y, end.x, end.y, greenPaint)
            }
            if (!writePngSafely(file, fallbackBitmap)) {
                return Result(false, "绿色兜底边界写入失败")
            }
            val generation = FloorPlanGenerationInfo(
                outlineClosed = true,
                outlineWidthPixels = vertices.maxOf { it.x } - vertices.minOf { it.x },
                outlineHeightPixels = vertices.maxOf { it.y } - vertices.minOf { it.y },
                outlineLeftPixels = vertices.minOf { it.x },
                outlineTopPixels = vertices.minOf { it.y },
                outlineRightPixels = vertices.maxOf { it.x },
                outlineBottomPixels = vertices.maxOf { it.y },
                rotationDegrees = Math.toDegrees(
                    atan2(fitted.longAxisY, fitted.longAxisX).toDouble()
                ).toFloat(),
                supportRatio = 0f,
                vertexCount = vertices.size,
                closeSizePixels = 0,
                dimensionCenterXPixels = fitted.centerX,
                dimensionCenterYPixels = fitted.centerY,
                dimensionLongAxisX = fitted.longAxisX,
                dimensionLongAxisY = fitted.longAxisY,
                dimensionShortAxisX = fitted.shortAxisX,
                dimensionShortAxisY = fitted.shortAxisY,
                dimensionLongSizePixels = fitted.longSize,
                dimensionShortSizePixels = fitted.shortSize,
                outlineVerticesPixels = vertices,
                footprintAreaPixelsSquared = fitted.longSize * fitted.shortSize,
                footprintPerimeterPixels = 2f * (fitted.longSize + fitted.shortSize)
            )
            annotateFile(file, generation, metersPerPixel)
        } catch (_: OutOfMemoryError) {
            Result(false, "图片过大，兜底标注内存不足")
        } catch (e: Exception) {
            Result(false, e.message ?: "户型图兜底标注失败")
        } finally {
            greenBase?.let { if (!it.isRecycled) it.recycle() }
            source?.let { if (!it.isRecycled) it.recycle() }
        }
    }

    private data class FittedRectangle(
        val vertices: List<FloorPlanPixelPoint>,
        val centerX: Float,
        val centerY: Float,
        val longAxisX: Float,
        val longAxisY: Float,
        val shortAxisX: Float,
        val shortAxisY: Float,
        val longSize: Float,
        val shortSize: Float
    )

    private fun largestConnectedWallPoints(
        darkMask: BooleanArray,
        width: Int,
        height: Int,
        pixelStep: Int
    ): List<FloorPlanPixelPoint> {
        // A small physical/image-relative closing connects broken samples on an
        // outside wall without bridging normal room-width gaps.
        val radius = (min(width, height) / 280f).roundToInt().coerceIn(2, 6)
        val integral = IntArray((width + 1) * (height + 1))
        for (y in 0 until height) {
            var rowSum = 0
            for (x in 0 until width) {
                if (darkMask[y * width + x]) rowSum++
                integral[(y + 1) * (width + 1) + x + 1] =
                    integral[y * (width + 1) + x + 1] + rowSum
            }
        }
        fun hasDark(left: Int, top: Int, right: Int, bottom: Int): Boolean {
            val stride = width + 1
            val sum = integral[bottom * stride + right] -
                integral[top * stride + right] -
                integral[bottom * stride + left] +
                integral[top * stride + left]
            return sum > 0
        }
        val expanded = BooleanArray(width * height)
        for (y in 0 until height) {
            val top = max(0, y - radius)
            val bottom = min(height, y + radius + 1)
            for (x in 0 until width) {
                expanded[y * width + x] = hasDark(
                    max(0, x - radius),
                    top,
                    min(width, x + radius + 1),
                    bottom
                )
            }
        }
        val labels = IntArray(width * height)
        val queue = IntArray(width * height)
        var label = 0
        var largestLabel = 0
        var largestCount = 0
        for (start in expanded.indices) {
            if (!expanded[start] || labels[start] != 0) continue
            label++
            var head = 0
            var tail = 0
            queue[tail++] = start
            labels[start] = label
            while (head < tail) {
                val current = queue[head++]
                val x = current % width
                val y = current / width
                for (dy in -1..1) {
                    val nextY = y + dy
                    if (nextY !in 0 until height) continue
                    for (dx in -1..1) {
                        if (dx == 0 && dy == 0) continue
                        val nextX = x + dx
                        if (nextX !in 0 until width) continue
                        val next = nextY * width + nextX
                        if (expanded[next] && labels[next] == 0) {
                            labels[next] = label
                            queue[tail++] = next
                        }
                    }
                }
            }
            if (tail > largestCount) {
                largestCount = tail
                largestLabel = label
            }
        }
        if (largestLabel == 0) return emptyList()
        val points = ArrayList<FloorPlanPixelPoint>()
        for (index in darkMask.indices) {
            if (!darkMask[index] || labels[index] != largestLabel) continue
            points += FloorPlanPixelPoint(
                (index % width * pixelStep).toFloat(),
                (index / width * pixelStep).toFloat()
            )
        }
        return points
    }

    private fun convexHull(points: List<FloorPlanPixelPoint>): List<FloorPlanPixelPoint> {
        if (points.size < 3) return points
        val sorted = points.distinctBy { it.x to it.y }
            .sortedWith(compareBy<FloorPlanPixelPoint> { it.x }.thenBy { it.y })
        if (sorted.size < 3) return sorted
        fun cross(
            origin: FloorPlanPixelPoint,
            a: FloorPlanPixelPoint,
            b: FloorPlanPixelPoint
        ): Float = (a.x - origin.x) * (b.y - origin.y) -
            (a.y - origin.y) * (b.x - origin.x)
        val lower = ArrayList<FloorPlanPixelPoint>()
        for (point in sorted) {
            while (lower.size >= 2 &&
                cross(lower[lower.lastIndex - 1], lower.last(), point) <= 0f) {
                lower.removeAt(lower.lastIndex)
            }
            lower += point
        }
        val upper = ArrayList<FloorPlanPixelPoint>()
        for (index in sorted.indices.reversed()) {
            val point = sorted[index]
            while (upper.size >= 2 &&
                cross(upper[upper.lastIndex - 1], upper.last(), point) <= 0f) {
                upper.removeAt(upper.lastIndex)
            }
            upper += point
        }
        lower.removeAt(lower.lastIndex)
        upper.removeAt(upper.lastIndex)
        return lower + upper
    }

    private fun minimumAreaRectangle(
        hull: List<FloorPlanPixelPoint>
    ): FittedRectangle? {
        if (hull.size < 3) return null
        var bestArea = Float.POSITIVE_INFINITY
        var best: FittedRectangle? = null
        hull.forEachIndexed { index, start ->
            val end = hull[(index + 1) % hull.size]
            val edgeLength = hypot(end.x - start.x, end.y - start.y)
            if (edgeLength <= 1e-3f) return@forEachIndexed
            var ux = (end.x - start.x) / edgeLength
            var uy = (end.y - start.y) / edgeLength
            var vx = -uy
            var vy = ux
            var minU = Float.POSITIVE_INFINITY
            var maxU = Float.NEGATIVE_INFINITY
            var minV = Float.POSITIVE_INFINITY
            var maxV = Float.NEGATIVE_INFINITY
            for (point in hull) {
                val u = point.x * ux + point.y * uy
                val v = point.x * vx + point.y * vy
                minU = min(minU, u)
                maxU = max(maxU, u)
                minV = min(minV, v)
                maxV = max(maxV, v)
            }
            var sizeU = maxU - minU
            var sizeV = maxV - minV
            val area = sizeU * sizeV
            if (!area.isFinite() || area >= bestArea || sizeU <= 2f || sizeV <= 2f) {
                return@forEachIndexed
            }
            if (sizeU < sizeV) {
                val oldUx = ux
                val oldUy = uy
                ux = vx
                uy = vy
                vx = -oldUx
                vy = -oldUy
                val oldMin = minU
                val oldMax = maxU
                minU = minV
                maxU = maxV
                minV = -oldMax
                maxV = -oldMin
                sizeU = maxU - minU
                sizeV = maxV - minV
            }
            fun point(u: Float, v: Float) = FloorPlanPixelPoint(
                ux * u + vx * v,
                uy * u + vy * v
            )
            val centerU = (minU + maxU) * 0.5f
            val centerV = (minV + maxV) * 0.5f
            val center = point(centerU, centerV)
            bestArea = area
            best = FittedRectangle(
                vertices = listOf(
                    point(minU, minV),
                    point(maxU, minV),
                    point(maxU, maxV),
                    point(minU, maxV)
                ),
                centerX = center.x,
                centerY = center.y,
                longAxisX = ux,
                longAxisY = uy,
                shortAxisX = vx,
                shortAxisY = vy,
                longSize = sizeU,
                shortSize = sizeV
            )
        }
        return best
    }

    private data class PreparedSource(
        val bitmap: Bitmap,
        val geometry: OrientedGeometry
    )

    private data class OrientedGeometry(
        val outlineBounds: RectF,
        val outlineVertices: List<FloorPlanPixelPoint>,
        val centerX: Float,
        val centerY: Float,
        val longAxisX: Float,
        val longAxisY: Float,
        val shortAxisX: Float,
        val shortAxisY: Float,
        val longSizePixels: Float,
        val shortSizePixels: Float
    )

    private fun canonicalizeAndCrop(
        source: Bitmap,
        geometry: OrientedGeometry
    ): PreparedSource {
        val longAxisAngle = Math.toDegrees(
            atan2(geometry.longAxisY.toDouble(), geometry.longAxisX.toDouble())
        ).toFloat()
        // Use one canonical report frame regardless of arbitrary SLAM yaw.
        val correctionDegrees = -longAxisAngle

        val canonical: Bitmap
        val canonicalGeometry: OrientedGeometry
        if (kotlin.math.abs(correctionDegrees) <= 0.05f) {
            canonical = source
            canonicalGeometry = geometry
        } else {
            val radians = Math.toRadians(correctionDegrees.toDouble())
            val cosine = kotlin.math.cos(radians).toFloat()
            val sine = kotlin.math.sin(radians).toFloat()
            val centerX = source.width * 0.5f
            val centerY = source.height * 0.5f
            fun rotatePoint(x: Float, y: Float): FloorPlanPixelPoint {
                val dx = x - centerX
                val dy = y - centerY
                return FloorPlanPixelPoint(
                    centerX + cosine * dx - sine * dy,
                    centerY + sine * dx + cosine * dy
                )
            }

            val corners = listOf(
                rotatePoint(0f, 0f),
                rotatePoint(source.width.toFloat(), 0f),
                rotatePoint(source.width.toFloat(), source.height.toFloat()),
                rotatePoint(0f, source.height.toFloat())
            )
            val rotatedLeft = corners.minOf { it.x }
            val rotatedTop = corners.minOf { it.y }
            val rotatedRight = corners.maxOf { it.x }
            val rotatedBottom = corners.maxOf { it.y }
            val width = ceil(rotatedRight - rotatedLeft).toInt().coerceAtLeast(1)
            val height = ceil(rotatedBottom - rotatedTop).toInt().coerceAtLeast(1)
            canonical = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888)
            Canvas(canonical).apply {
                drawColor(source.getPixel(0, 0))
                translate(-rotatedLeft, -rotatedTop)
                rotate(correctionDegrees, centerX, centerY)
                drawBitmap(source, 0f, 0f, null)
            }
            fun canonicalPoint(point: FloorPlanPixelPoint): FloorPlanPixelPoint {
                val rotated = rotatePoint(point.x, point.y)
                return FloorPlanPixelPoint(
                    rotated.x - rotatedLeft,
                    rotated.y - rotatedTop
                )
            }
            val canonicalVertices = geometry.outlineVertices.map(::canonicalPoint)
            val canonicalCenter = canonicalPoint(
                FloorPlanPixelPoint(geometry.centerX, geometry.centerY)
            )
            val transformedLongX =
                cosine * geometry.longAxisX - sine * geometry.longAxisY
            val transformedLongY =
                sine * geometry.longAxisX + cosine * geometry.longAxisY
            val transformedShortX =
                cosine * geometry.shortAxisX - sine * geometry.shortAxisY
            val transformedShortY =
                sine * geometry.shortAxisX + cosine * geometry.shortAxisY
            canonicalGeometry = geometry.copy(
                outlineBounds = RectF(
                    canonicalVertices.minOf { it.x },
                    canonicalVertices.minOf { it.y },
                    canonicalVertices.maxOf { it.x },
                    canonicalVertices.maxOf { it.y }
                ),
                outlineVertices = canonicalVertices,
                centerX = canonicalCenter.x,
                centerY = canonicalCenter.y,
                // Snap the tiny floating-point residue as well; dimension
                // guides then share the exact horizontal/vertical axes used by
                // the rendered architectural outline.
                longAxisX = if (kotlin.math.abs(transformedLongX) < 1e-4f) 0f
                    else transformedLongX,
                longAxisY = if (kotlin.math.abs(transformedLongY) < 1e-4f) 0f
                    else transformedLongY,
                shortAxisX = if (kotlin.math.abs(transformedShortX) < 1e-4f) 0f
                    else transformedShortX,
                shortAxisY = if (kotlin.math.abs(transformedShortY) < 1e-4f) 0f
                    else transformedShortY
            )
        }

        // Preserve the complete rotated point-cloud canvas. Cropping to the
        // fitted outline discarded black observations whenever that outline
        // was conservative, most visibly along the bottom edge.
        return PreparedSource(canonical, canonicalGeometry)
    }

    private fun validatedGeometry(
        bitmap: Bitmap,
        generation: FloorPlanGenerationInfo
    ): OrientedGeometry? {
        val vertices = generation.outlineVerticesPixels.filter {
            it.x.isFinite() && it.y.isFinite() &&
                it.x in 0f..bitmap.width.toFloat() &&
                it.y in 0f..bitmap.height.toFloat()
        }
        if (vertices.size < 3) return null
        val left = vertices.minOf { it.x }
        val top = vertices.minOf { it.y }
        val right = vertices.maxOf { it.x }
        val bottom = vertices.maxOf { it.y }
        if (!left.isFinite() || !top.isFinite() || !right.isFinite() ||
            !bottom.isFinite() || right - left <= 1f || bottom - top <= 1f) {
            return null
        }
        var longAxisX = generation.dimensionLongAxisX
        var longAxisY = generation.dimensionLongAxisY
        val longNorm = hypot(longAxisX, longAxisY)
        val suppliedShortNorm = hypot(
            generation.dimensionShortAxisX,
            generation.dimensionShortAxisY
        )
        if (longNorm <= 1e-4f || !longNorm.isFinite() ||
            suppliedShortNorm <= 1e-4f || !suppliedShortNorm.isFinite()) {
            return null
        }
        longAxisX /= longNorm
        longAxisY /= longNorm
        var shortAxisX = -longAxisY
        var shortAxisY = longAxisX
        val suppliedShortX = generation.dimensionShortAxisX / suppliedShortNorm
        val suppliedShortY = generation.dimensionShortAxisY / suppliedShortNorm
        if (shortAxisX * suppliedShortX + shortAxisY * suppliedShortY < 0f) {
            shortAxisX = -shortAxisX
            shortAxisY = -shortAxisY
        }
        fun projectionRange(axisX: Float, axisY: Float): Pair<Float, Float> {
            var minimum = Float.POSITIVE_INFINITY
            var maximum = Float.NEGATIVE_INFINITY
            for (point in vertices) {
                val projection = point.x * axisX + point.y * axisY
                minimum = min(minimum, projection)
                maximum = max(maximum, projection)
            }
            return minimum to maximum
        }
        var longRange = projectionRange(longAxisX, longAxisY)
        var shortRange = projectionRange(shortAxisX, shortAxisY)
        if (longRange.second - longRange.first < shortRange.second - shortRange.first) {
            val oldLongX = longAxisX
            val oldLongY = longAxisY
            longAxisX = shortAxisX
            longAxisY = shortAxisY
            shortAxisX = -oldLongX
            shortAxisY = -oldLongY
            longRange = projectionRange(longAxisX, longAxisY)
            shortRange = projectionRange(shortAxisX, shortAxisY)
        }
        val longSize = longRange.second - longRange.first
        val shortSize = shortRange.second - shortRange.first
        if (longSize <= 1f || shortSize <= 1f) return null
        val centerLong = (longRange.first + longRange.second) * 0.5f
        val centerShort = (shortRange.first + shortRange.second) * 0.5f
        val centerX = longAxisX * centerLong + shortAxisX * centerShort
        val centerY = longAxisY * centerLong + shortAxisY * centerShort
        return OrientedGeometry(
            outlineBounds = RectF(left, top, right, bottom),
            outlineVertices = vertices,
            centerX = centerX,
            centerY = centerY,
            longAxisX = longAxisX,
            longAxisY = longAxisY,
            shortAxisX = shortAxisX,
            shortAxisY = shortAxisY,
            longSizePixels = longSize,
            shortSizePixels = shortSize
        )
    }

    private fun drawAnnotations(
        source: Bitmap,
        geometry: OrientedGeometry,
        lengthMeters: Float,
        widthMeters: Float,
        areaSquareMeters: Float,
        metersPerPixel: Float
    ): Bitmap {
        val shortSide = min(source.width, source.height).toFloat().coerceAtLeast(1f)
        // Compact report style: small labels sit directly beside the green
        // contour instead of surrounding the plan with large dimension lines.
        val textSize = (shortSide * 0.022f).coerceIn(9f, 24f)
        val summaryHeight = textSize * 1.9f
        val dimensionMargin = (textSize * 2.0f).coerceIn(22f, 64f)
        val outerPadding = max(10f, textSize * 0.45f)

        val summary = String.format(
            Locale.US,
            "长：%.2f m　宽：%.2f m　闭合轮廓面积：%.2f m²",
            lengthMeters,
            widthMeters,
            areaSquareMeters
        )
        val summaryPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.BLACK
            this.textSize = textSize
            textAlign = Paint.Align.CENTER
            style = Paint.Style.FILL
        }
        val requiredSummaryWidth = summaryPaint.measureText(summary) + outerPadding * 2f
        val contentWidth = max(source.width.toFloat(), requiredSummaryWidth)
        val outputWidth = ceil(contentWidth + dimensionMargin * 2f).toInt()
        val outputHeight = ceil(summaryHeight + source.height + dimensionMargin * 2f).toInt()
        val output = Bitmap.createBitmap(
            max(1, outputWidth),
            max(1, outputHeight),
            Bitmap.Config.ARGB_8888
        )
        val canvas = Canvas(output)
        canvas.drawColor(Color.WHITE)

        val sourceX = dimensionMargin + (contentWidth - source.width) * 0.5f
        val sourceY = summaryHeight + dimensionMargin
        canvas.drawBitmap(source, sourceX, sourceY, null)

        val headerPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.rgb(247, 247, 247)
            style = Paint.Style.FILL
        }
        val guidePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.rgb(70, 70, 70)
            strokeWidth = 1f
            style = Paint.Style.STROKE
        }
        val labelPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.BLACK
            this.textSize = textSize * 0.78f
            textAlign = Paint.Align.CENTER
            style = Paint.Style.FILL
        }
        val labelBackground = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.WHITE
            style = Paint.Style.FILL
        }
        canvas.drawRect(0f, 0f, output.width.toFloat(), summaryHeight, headerPaint)
        canvas.drawLine(0f, summaryHeight, output.width.toFloat(), summaryHeight, guidePaint)
        val summaryMetrics = summaryPaint.fontMetrics
        val summaryBaseline = summaryHeight * 0.5f -
            (summaryMetrics.ascent + summaryMetrics.descent) * 0.5f
        canvas.drawText(summary, output.width * 0.5f, summaryBaseline, summaryPaint)

        val dimensionOffset = textSize * 0.72f
        var signedAreaTwice = 0f
        geometry.outlineVertices.forEachIndexed { index, point ->
            val next = geometry.outlineVertices[
                (index + 1) % geometry.outlineVertices.size
            ]
            signedAreaTwice += point.x * next.y - next.x * point.y
        }
        // In bitmap coordinates (+Y downward), a positive signed area means
        // clockwise winding. Its outward normal is (dy, -dx).
        val outwardSign = if (signedAreaTwice >= 0f) 1f else -1f
        geometry.outlineVertices.forEachIndexed { index, start ->
            val end = geometry.outlineVertices[
                (index + 1) % geometry.outlineVertices.size
            ]
            val dx = end.x - start.x
            val dy = end.y - start.y
            val edgeLengthPixels = hypot(dx, dy)
            if (!edgeLengthPixels.isFinite() || edgeLengthPixels <= 1f) {
                return@forEachIndexed
            }
            val edgeLengthMeters = edgeLengthPixels * metersPerPixel
            if (!edgeLengthMeters.isFinite()) {
                return@forEachIndexed
            }
            val normalX = outwardSign * dy / edgeLengthPixels
            val normalY = outwardSign * -dx / edgeLengthPixels
            if (edgeLengthMeters >= MIN_ANNOTATED_EDGE_METERS) {
                drawContourLabel(
                    canvas = canvas,
                    startX = sourceX + start.x + normalX * dimensionOffset,
                    startY = sourceY + start.y + normalY * dimensionOffset,
                    endX = sourceX + end.x + normalX * dimensionOffset,
                    endY = sourceY + end.y + normalY * dimensionOffset,
                    label = String.format(Locale.US, "%.2f m", edgeLengthMeters),
                    textPaint = labelPaint,
                    textBackgroundPaint = labelBackground
                )
            }
        }
        return output
    }

    private fun drawContourLabel(
        canvas: Canvas,
        startX: Float,
        startY: Float,
        endX: Float,
        endY: Float,
        label: String,
        textPaint: Paint,
        textBackgroundPaint: Paint
    ) {
        val midX = (startX + endX) * 0.5f
        val midY = (startY + endY) * 0.5f
        var labelAngle = Math.toDegrees(
            atan2(endY - startY, endX - startX).toDouble()
        ).toFloat()
        while (labelAngle > 90f) labelAngle -= 180f
        while (labelAngle <= -90f) labelAngle += 180f
        canvas.save()
        canvas.rotate(labelAngle, midX, midY)
        val metrics = textPaint.fontMetrics
        val baseline = midY - (metrics.ascent + metrics.descent) * 0.5f
        val textWidth = textPaint.measureText(label)
        val padX = textPaint.textSize * 0.18f
        val padY = textPaint.textSize * 0.08f
        canvas.drawRoundRect(
            RectF(
                midX - textWidth * 0.5f - padX,
                baseline + metrics.ascent - padY,
                midX + textWidth * 0.5f + padX,
                baseline + metrics.descent + padY
            ),
            padY,
            padY,
            textBackgroundPaint
        )
        canvas.drawText(label, midX, baseline, textPaint)
        canvas.restore()
    }

    private fun drawDoubleArrow(
        canvas: Canvas,
        startX: Float,
        startY: Float,
        endX: Float,
        endY: Float,
        label: String,
        linePaint: Paint,
        textPaint: Paint,
        textBackgroundPaint: Paint,
        arrowLength: Float
    ) {
        canvas.drawLine(startX, startY, endX, endY, linePaint)
        val length = hypot(endX - startX, endY - startY)
        if (length <= 1f) return
        val ux = (endX - startX) / length
        val uy = (endY - startY) / length
        val px = -uy
        val py = ux
        val arrowHalfWidth = arrowLength * 0.46f
        canvas.drawLine(startX, startY,
            startX + ux * arrowLength + px * arrowHalfWidth,
            startY + uy * arrowLength + py * arrowHalfWidth, linePaint)
        canvas.drawLine(startX, startY,
            startX + ux * arrowLength - px * arrowHalfWidth,
            startY + uy * arrowLength - py * arrowHalfWidth, linePaint)
        canvas.drawLine(endX, endY,
            endX - ux * arrowLength + px * arrowHalfWidth,
            endY - uy * arrowLength + py * arrowHalfWidth, linePaint)
        canvas.drawLine(endX, endY,
            endX - ux * arrowLength - px * arrowHalfWidth,
            endY - uy * arrowLength - py * arrowHalfWidth, linePaint)

        if (label.isBlank()) return

        val midX = (startX + endX) * 0.5f
        val midY = (startY + endY) * 0.5f
        var labelAngle = Math.toDegrees(
            atan2(endY - startY, endX - startX).toDouble()
        ).toFloat()
        while (labelAngle > 90f) labelAngle -= 180f
        while (labelAngle <= -90f) labelAngle += 180f
        canvas.save()
        canvas.rotate(labelAngle, midX, midY)
        val metrics = textPaint.fontMetrics
        val baseline = midY - (metrics.ascent + metrics.descent) * 0.5f
        val textWidth = textPaint.measureText(label)
        val padX = textPaint.textSize * 0.28f
        val padY = textPaint.textSize * 0.16f
        canvas.drawRoundRect(
            RectF(
                midX - textWidth * 0.5f - padX,
                baseline + metrics.ascent - padY,
                midX + textWidth * 0.5f + padX,
                baseline + metrics.descent + padY
            ),
            padY,
            padY,
            textBackgroundPaint
        )
        canvas.drawText(label, midX, baseline, textPaint)
        canvas.restore()
    }

    private fun writePngSafely(file: File, bitmap: Bitmap): Boolean {
        val temporary = File(file.parentFile, "${file.name}.annotating.tmp")
        return try {
            FileOutputStream(temporary).use { stream ->
                if (!bitmap.compress(Bitmap.CompressFormat.PNG, 100, stream)) {
                    throw IllegalStateException("PNG压缩失败")
                }
            }
            temporary.copyTo(file, overwrite = true)
            temporary.delete()
            file.exists() && file.length() > 0L
        } catch (_: Exception) {
            temporary.delete()
            false
        }
    }
}
