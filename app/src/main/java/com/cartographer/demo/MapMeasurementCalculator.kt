package com.cartographer.demo

import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin
import kotlin.math.hypot

data class MapMeasurementPoint(val x: Float, val y: Float)

data class MapMeasurement(
    val minX: Float,
    val maxX: Float,
    val minY: Float,
    val maxY: Float,
    val xSizeMeters: Float,
    val ySizeMeters: Float,
    val lengthMeters: Float,
    val widthMeters: Float,
    val boundingAreaSquareMeters: Float,
    val orientedCenterX: Float = Float.NaN,
    val orientedCenterY: Float = Float.NaN,
    val longAxisX: Float = Float.NaN,
    val longAxisY: Float = Float.NaN,
    val shortAxisX: Float = Float.NaN,
    val shortAxisY: Float = Float.NaN,
    val orientedLongSizeMeters: Float = Float.NaN,
    val orientedShortSizeMeters: Float = Float.NaN,
    val outlineWorld: List<MapMeasurementPoint> = emptyList()
) {
    fun hasSharedFloorPlanGeometry(): Boolean = outlineWorld.size >= 3 &&
        orientedCenterX.isFinite() && orientedCenterY.isFinite() &&
        longAxisX.isFinite() && longAxisY.isFinite() &&
        shortAxisX.isFinite() && shortAxisY.isFinite() &&
        orientedLongSizeMeters.isFinite() && orientedLongSizeMeters > 0f &&
        orientedShortSizeMeters.isFinite() && orientedShortSizeMeters > 0f
}

object MapMeasurementCalculator {
    fun calculate(textures: List<SubmapTexture>): MapMeasurement? {
        var minX = Float.POSITIVE_INFINITY
        var minY = Float.POSITIVE_INFINITY
        var maxX = Float.NEGATIVE_INFINITY
        var maxY = Float.NEGATIVE_INFINITY
        var hasValidCell = false

        for (texture in textures) {
            if (texture.width <= 0 || texture.height <= 0 ||
                !texture.resolution.isFinite() || texture.resolution <= 0f) {
                continue
            }
            val resolution = texture.resolution
            val cosTheta = cos(texture.theta).toFloat()
            val sinTheta = sin(texture.theta).toFloat()
            val stepXWorldX = sinTheta * resolution
            val stepXWorldY = -cosTheta * resolution
            val stepYWorldX = -cosTheta * resolution
            val stepYWorldY = -sinTheta * resolution

            // Include the whole raster cell, not just its top-left sample.
            val cellOffsetX = floatArrayOf(
                0f,
                stepXWorldX,
                stepYWorldX,
                stepXWorldX + stepYWorldX
            )
            val cellOffsetY = floatArrayOf(
                0f,
                stepXWorldY,
                stepYWorldY,
                stepXWorldY + stepYWorldY
            )
            val cellMinOffsetX = cellOffsetX.minOrNull() ?: 0f
            val cellMaxOffsetX = cellOffsetX.maxOrNull() ?: 0f
            val cellMinOffsetY = cellOffsetY.minOrNull() ?: 0f
            val cellMaxOffsetY = cellOffsetY.maxOrNull() ?: 0f

            for (py in 0 until texture.height) {
                val rowWorldX = texture.originX + py * stepYWorldX
                val rowWorldY = texture.originY + py * stepYWorldY
                val rowOffset = py * texture.width
                for (px in 0 until texture.width) {
                    val pixelIndex = rowOffset + px
                    if (pixelIndex >= texture.pixels.size) break
                    // Match the floor-plan algorithm input. Low-alpha cells
                    // are explored/free-space or transient returns and must
                    // not enlarge the building dimensions.
                    if (texture.pixels[pixelIndex] ushr 24 <
                        FloorPlanMapExporter.STABLE_WALL_MIN_ALPHA) continue

                    val worldX = rowWorldX + px * stepXWorldX
                    val worldY = rowWorldY + px * stepXWorldY
                    minX = min(minX, worldX + cellMinOffsetX)
                    maxX = max(maxX, worldX + cellMaxOffsetX)
                    minY = min(minY, worldY + cellMinOffsetY)
                    maxY = max(maxY, worldY + cellMaxOffsetY)
                    hasValidCell = true
                }
            }
        }

        if (!hasValidCell || !minX.isFinite() || !maxX.isFinite() ||
            !minY.isFinite() || !maxY.isFinite()) {
            return null
        }
        val xSize = (maxX - minX).coerceAtLeast(0f)
        val ySize = (maxY - minY).coerceAtLeast(0f)
        return MapMeasurement(
            minX = minX,
            maxX = maxX,
            minY = minY,
            maxY = maxY,
            xSizeMeters = xSize,
            ySizeMeters = ySize,
            lengthMeters = max(xSize, ySize),
            widthMeters = min(xSize, ySize),
            boundingAreaSquareMeters = xSize * ySize
        )
    }

    /**
     * Builds the map overlay from the exact polygon and rotated rectangle
     * returned by the floor-plan algorithm. Export pixels use +Y down while
     * the SLAM world uses +Y up, hence the Y sign inversion below.
     */
    fun fromFloorPlan(
        generation: FloorPlanGenerationInfo,
        geometry: FloorPlanMapExporter.ExportGeometry
    ): MapMeasurement? {
        val resolution = geometry.resolutionMetersPerPixel
        if (!resolution.isFinite() || resolution <= 0f ||
            generation.outlineVerticesPixels.size < 3) return null

        val outline = generation.outlineVerticesPixels.map { point ->
            MapMeasurementPoint(
                x = geometry.worldMinX + point.x * resolution,
                y = geometry.worldMaxY - point.y * resolution
            )
        }
        if (outline.any { !it.x.isFinite() || !it.y.isFinite() }) return null

        val minX = outline.minOf { it.x }
        val maxX = outline.maxOf { it.x }
        val minY = outline.minOf { it.y }
        val maxY = outline.maxOf { it.y }
        val xSize = (maxX - minX).coerceAtLeast(0f)
        val ySize = (maxY - minY).coerceAtLeast(0f)

        var longAxisX = generation.dimensionLongAxisX
        var longAxisY = -generation.dimensionLongAxisY
        val longNorm = hypot(longAxisX, longAxisY)
        var shortAxisX = generation.dimensionShortAxisX
        var shortAxisY = -generation.dimensionShortAxisY
        val shortNorm = hypot(shortAxisX, shortAxisY)
        if (!longNorm.isFinite() || longNorm <= 1e-4f ||
            !shortNorm.isFinite() || shortNorm <= 1e-4f) return null
        longAxisX /= longNorm
        longAxisY /= longNorm
        shortAxisX /= shortNorm
        shortAxisY /= shortNorm

        fun projectionRange(axisX: Float, axisY: Float): Pair<Float, Float> {
            var minimum = Float.POSITIVE_INFINITY
            var maximum = Float.NEGATIVE_INFINITY
            for (point in outline) {
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
        val centerLong = (longRange.first + longRange.second) * 0.5f
        val centerShort = (shortRange.first + shortRange.second) * 0.5f
        val orientedCenterX = longAxisX * centerLong + shortAxisX * centerShort
        val orientedCenterY = longAxisY * centerLong + shortAxisY * centerShort
        val footprintArea = generation.footprintAreaPixelsSquared *
            resolution * resolution
        if (!longSize.isFinite() || !shortSize.isFinite() ||
            !footprintArea.isFinite() || longSize <= 0f || shortSize <= 0f ||
            footprintArea <= 0f) return null

        return MapMeasurement(
            minX = minX,
            maxX = maxX,
            minY = minY,
            maxY = maxY,
            xSizeMeters = xSize,
            ySizeMeters = ySize,
            lengthMeters = longSize,
            widthMeters = shortSize,
            boundingAreaSquareMeters = footprintArea,
            orientedCenterX = orientedCenterX,
            orientedCenterY = orientedCenterY,
            longAxisX = longAxisX,
            longAxisY = longAxisY,
            shortAxisX = shortAxisX,
            shortAxisY = shortAxisY,
            orientedLongSizeMeters = longSize,
            orientedShortSizeMeters = shortSize,
            outlineWorld = outline
        )
    }
}
