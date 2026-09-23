package com.cartographer.demo

import android.graphics.Bitmap
import android.graphics.Color
import java.util.PriorityQueue
import kotlin.math.ceil
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.roundToInt

/** Calculates the wall-aware area supported by RSSI samples taken along the route. */
object HeatMapCoverageCalculator {
    const val SUPPORT_RADIUS_METERS = 1.5f
    private const val TARGET_GRID_METERS = 0.10f
    private const val MAX_GRID_SIDE = 512
    private const val MAX_SAMPLE_GAP_MILLIS = 3_000L
    private const val MAX_SAMPLE_JUMP_METERS = 3.0f

    data class Result(
        val coveragePercent: Float,
        val coveredAreaSquareMeters: Float,
        val targetAreaSquareMeters: Float,
        val supportRadiusMeters: Float,
        val validSampleCount: Int,
        /** Opaque only where heat-map interpolation has reliable route support. */
        val supportMask: Bitmap
    )

    private data class GridNode(val index: Int, val distanceMeters: Float)

    fun calculate(
        samples: List<RssiSample>,
        geometry: FloorPlanMapExporter.ExportGeometry,
        outlinePixels: List<FloorPlanPixelPoint>,
        semanticMap: Bitmap
    ): Result? {
        if (outlinePixels.size < 3 || geometry.widthPx <= 0 || geometry.heightPx <= 0 ||
            semanticMap.width != geometry.widthPx || semanticMap.height != geometry.heightPx ||
            !geometry.resolutionMetersPerPixel.isFinite() ||
            geometry.resolutionMetersPerPixel <= 0f) return null

        val validSamples = samples.filter {
            it.worldX.isFinite() && it.worldY.isFinite() &&
                it.rssiDbm.isFinite() && it.rssiDbm in -150f..0f
        }
        if (validSamples.isEmpty()) return null

        val resolution = geometry.resolutionMetersPerPixel
        val desiredStep = max(1, (TARGET_GRID_METERS / resolution).roundToInt())
        val sizeLimitedStep = max(
            ceil(geometry.widthPx.toDouble() / MAX_GRID_SIDE).toInt(),
            ceil(geometry.heightPx.toDouble() / MAX_GRID_SIDE).toInt()
        )
        val pixelStep = max(desiredStep, sizeLimitedStep)
        val gridWidth = ceil(geometry.widthPx.toDouble() / pixelStep).toInt()
        val gridHeight = ceil(geometry.heightPx.toDouble() / pixelStep).toInt()
        val gridSize = gridWidth * gridHeight
        val cellMeters = pixelStep * resolution

        val semanticPixels = IntArray(semanticMap.width * semanticMap.height)
        semanticMap.getPixels(
            semanticPixels, 0, semanticMap.width, 0, 0,
            semanticMap.width, semanticMap.height
        )
        val wallIntegral = IntArray((semanticMap.width + 1) * (semanticMap.height + 1))
        val integralStride = semanticMap.width + 1
        for (y in 0 until semanticMap.height) {
            var rowWalls = 0
            for (x in 0 until semanticMap.width) {
                val color = semanticPixels[y * semanticMap.width + x]
                if (Color.red(color) < 80 && Color.green(color) < 80 &&
                    Color.blue(color) < 80) rowWalls++
                wallIntegral[(y + 1) * integralStride + x + 1] =
                    wallIntegral[y * integralStride + x + 1] + rowWalls
            }
        }
        fun containsWall(left: Int, top: Int, right: Int, bottom: Int): Boolean {
            val count = wallIntegral[bottom * integralStride + right] -
                wallIntegral[top * integralStride + right] -
                wallIntegral[bottom * integralStride + left] +
                wallIntegral[top * integralStride + left]
            return count > 0
        }
        fun insideOutline(x: Float, y: Float): Boolean {
            var inside = false
            var previous = outlinePixels.last()
            for (current in outlinePixels) {
                if ((current.y > y) != (previous.y > y)) {
                    val crossingX = (previous.x - current.x) * (y - current.y) /
                        (previous.y - current.y) + current.x
                    if (x < crossingX) inside = !inside
                }
                previous = current
            }
            return inside
        }

        val traversable = BooleanArray(gridSize)
        var targetCellCount = 0
        for (gridY in 0 until gridHeight) {
            val top = gridY * pixelStep
            val bottom = ((gridY + 1) * pixelStep).coerceAtMost(semanticMap.height)
            val centerY = (top + bottom) * 0.5f
            for (gridX in 0 until gridWidth) {
                val left = gridX * pixelStep
                val right = ((gridX + 1) * pixelStep).coerceAtMost(semanticMap.width)
                val centerX = (left + right) * 0.5f
                val index = gridY * gridWidth + gridX
                traversable[index] = insideOutline(centerX, centerY) &&
                    !containsWall(left, top, right, bottom)
                if (traversable[index]) targetCellCount++
            }
        }
        if (targetCellCount == 0) return null

        val seed = BooleanArray(gridSize)
        fun addSeedPixel(pixelX: Float, pixelY: Float) {
            val centerX = (pixelX / pixelStep).toInt().coerceIn(0, gridWidth - 1)
            val centerY = (pixelY / pixelStep).toInt().coerceIn(0, gridHeight - 1)
            var bestIndex = -1
            var bestDistance = Int.MAX_VALUE
            for (dy in -2..2) {
                val y = centerY + dy
                if (y !in 0 until gridHeight) continue
                for (dx in -2..2) {
                    val x = centerX + dx
                    if (x !in 0 until gridWidth) continue
                    val index = y * gridWidth + x
                    val distance = dx * dx + dy * dy
                    if (traversable[index] && distance < bestDistance) {
                        bestIndex = index
                        bestDistance = distance
                    }
                }
            }
            if (bestIndex >= 0) seed[bestIndex] = true
        }
        fun pixelPoint(sample: RssiSample) = FloorPlanPixelPoint(
            geometry.worldToPixelX(sample.worldX),
            geometry.worldToPixelY(sample.worldY)
        )

        validSamples.forEach { sample ->
            val point = pixelPoint(sample)
            if (point.x in 0f..<geometry.widthPx.toFloat() &&
                point.y in 0f..<geometry.heightPx.toFloat()) {
                addSeedPixel(point.x, point.y)
            }
        }
        for (index in 1 until validSamples.size) {
            val first = validSamples[index - 1]
            val second = validSamples[index]
            val elapsed = second.timestampMillis - first.timestampMillis
            val timeContinuous = if (first.timestampMillis > 0L && second.timestampMillis > 0L) {
                elapsed in 0..MAX_SAMPLE_GAP_MILLIS
            } else {
                true
            }
            val distance = hypot(second.worldX - first.worldX, second.worldY - first.worldY)
            if (!timeContinuous || distance > MAX_SAMPLE_JUMP_METERS) continue
            val firstPixel = pixelPoint(first)
            val secondPixel = pixelPoint(second)
            val interpolationCount = max(1, ceil(distance / (cellMeters * 0.5f)).toInt())
            for (step in 0..interpolationCount) {
                val fraction = step.toFloat() / interpolationCount
                addSeedPixel(
                    firstPixel.x + (secondPixel.x - firstPixel.x) * fraction,
                    firstPixel.y + (secondPixel.y - firstPixel.y) * fraction
                )
            }
        }
        if (seed.none { it }) return null

        val distances = FloatArray(gridSize) { Float.POSITIVE_INFINITY }
        val queue = PriorityQueue<GridNode>(compareBy(GridNode::distanceMeters))
        seed.forEachIndexed { index, isSeed ->
            if (isSeed) {
                distances[index] = 0f
                queue += GridNode(index, 0f)
            }
        }
        val directions = arrayOf(
            intArrayOf(-1, 0), intArrayOf(1, 0), intArrayOf(0, -1), intArrayOf(0, 1),
            intArrayOf(-1, -1), intArrayOf(1, -1), intArrayOf(-1, 1), intArrayOf(1, 1)
        )
        while (queue.isNotEmpty()) {
            val current = queue.remove()
            if (current.distanceMeters != distances[current.index] ||
                current.distanceMeters >= SUPPORT_RADIUS_METERS) continue
            val x = current.index % gridWidth
            val y = current.index / gridWidth
            for ((dx, dy) in directions) {
                val nextX = x + dx
                val nextY = y + dy
                if (nextX !in 0 until gridWidth || nextY !in 0 until gridHeight) continue
                val nextIndex = nextY * gridWidth + nextX
                if (!traversable[nextIndex]) continue
                if (dx != 0 && dy != 0 &&
                    (!traversable[y * gridWidth + nextX] ||
                        !traversable[nextY * gridWidth + x])) continue
                val stepDistance = cellMeters * if (dx != 0 && dy != 0) 1.41421356f else 1f
                val nextDistance = current.distanceMeters + stepDistance
                if (nextDistance <= SUPPORT_RADIUS_METERS && nextDistance < distances[nextIndex]) {
                    distances[nextIndex] = nextDistance
                    queue += GridNode(nextIndex, nextDistance)
                }
            }
        }

        var coveredCellCount = 0
        val maskPixels = IntArray(gridSize)
        for (index in distances.indices) {
            if (traversable[index] && distances[index] <= SUPPORT_RADIUS_METERS) {
                maskPixels[index] = Color.WHITE
                coveredCellCount++
            } else {
                maskPixels[index] = Color.TRANSPARENT
            }
        }
        val coarseMask = Bitmap.createBitmap(
            maskPixels, gridWidth, gridHeight, Bitmap.Config.ARGB_8888
        )
        val supportMask = Bitmap.createScaledBitmap(
            coarseMask, geometry.widthPx, geometry.heightPx, false
        )
        if (supportMask !== coarseMask) coarseMask.recycle()
        val cellArea = cellMeters * cellMeters
        return Result(
            coveragePercent = 100f * coveredCellCount / targetCellCount,
            coveredAreaSquareMeters = coveredCellCount * cellArea,
            targetAreaSquareMeters = targetCellCount * cellArea,
            supportRadiusMeters = SUPPORT_RADIUS_METERS,
            validSampleCount = validSamples.size,
            supportMask = supportMask
        )
    }
}
