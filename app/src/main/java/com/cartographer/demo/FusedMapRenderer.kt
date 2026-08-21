package com.cartographer.demo

import android.graphics.Bitmap
import android.graphics.Color
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.ceil
import kotlin.math.cos
import kotlin.math.exp
import kotlin.math.floor
import kotlin.math.hypot
import kotlin.math.ln
import kotlin.math.max
import kotlin.math.min
import kotlin.math.roundToInt
import kotlin.math.sin

/**
 * Projects every Cartographer submap into one world-aligned confidence grid.
 *
 * This is a display renderer only. It does not feed the fused pixels back into
 * Cartographer and therefore cannot alter scan matching, pose graph constraints
 * or the serialized pbstream.
 */
object FusedMapRenderer {
    private const val MAX_SIDE_PX = 1600
    private const val MAX_LIVE_SUBMAPS_PER_TRAJECTORY = 16
    private const val LIVE_RECENT_SUBMAPS_PER_TRAJECTORY = 4
    private const val PADDING_CELLS = 6
    private const val FREE_MAX_ALPHA = 54
    private const val MIN_WALL_ALPHA = 55
    private const val MIN_PROBABILITY = 0.10f
    private const val MAX_PROBABILITY = 0.90f
    private const val MIN_OCCUPIED_PROBABILITY = 0.68f
    private const val RELIABLE_OCCUPIED_PROBABILITY = 0.76f
    private val MIN_LOG_ODDS = ln(
        MIN_PROBABILITY / (1f - MIN_PROBABILITY)
    )
    private val MAX_LOG_ODDS = ln(
        MAX_PROBABILITY / (1f - MAX_PROBABILITY)
    )

    data class Result(
        val bitmap: Bitmap,
        val resolutionMetersPerPixel: Float,
        val worldMinX: Float,
        val worldMaxY: Float,
        val contentMinX: Float,
        val contentMaxX: Float,
        val contentMinY: Float,
        val contentMaxY: Float,
        // Final wall and explored-space masks are exposed so floor-plan export can
        // consume the exact finalized fusion used by the map display instead
        // of independently compositing raw submaps with different thresholds.
        val stableWallMask: BooleanArray,
        val visibleWallMask: BooleanArray,
        val freeSpaceMask: BooleanArray,
        // Neutral gray/white/black occupancy pixels fused from untouched
        // Cartographer intensity/alpha, independent of the live highlight.
        val occupancyPixels: IntArray
    )

    fun render(
        textures: List<SubmapTexture>,
        finalized: Boolean = false
    ): Result? {
        val allValid = textures.filter {
            it.width > 0 && it.height > 0 &&
                it.resolution.isFinite() && it.resolution > 0f &&
                it.originX.isFinite() && it.originY.isFinite() &&
                it.theta.isFinite() &&
                it.pixels.size >= it.width * it.height
        }
        if (allValid.isEmpty()) return null

        // Consecutive 2D submaps overlap by roughly half. A proportional
        // "every second submap" policy still grows without bound and eventually
        // makes a long mapping session spend all of its time repainting history.
        // Keep a fixed, uniformly distributed history plus the newest active
        // submaps. Final rendering still uses every submap.
        val valid = if (finalized) {
            allValid
        } else {
            allValid.groupBy { it.trajectoryId }.values.flatMap { trajectory ->
                selectLiveSubmaps(trajectory)
            }
        }

        var minX = Float.POSITIVE_INFINITY
        var minY = Float.POSITIVE_INFINITY
        var maxX = Float.NEGATIVE_INFINITY
        var maxY = Float.NEGATIVE_INFINITY
        for (texture in valid) {
            val corners = arrayOf(
                texturePixelToWorld(texture, 0f, 0f),
                texturePixelToWorld(texture, texture.width.toFloat(), 0f),
                texturePixelToWorld(texture, 0f, texture.height.toFloat()),
                texturePixelToWorld(
                    texture,
                    texture.width.toFloat(),
                    texture.height.toFloat()
                )
            )
            for ((x, y) in corners) {
                minX = min(minX, x)
                minY = min(minY, y)
                maxX = max(maxX, x)
                maxY = max(maxY, y)
            }
        }
        if (!minX.isFinite() || !minY.isFinite() ||
            !maxX.isFinite() || !maxY.isFinite() ||
            maxX <= minX || maxY <= minY) {
            return null
        }

        var resolution = valid.minOf { it.resolution.toDouble() }.toFloat()
        var width = ceil((maxX - minX) / resolution).toInt() + PADDING_CELLS * 2
        var height = ceil((maxY - minY) / resolution).toInt() + PADDING_CELLS * 2
        val initialMaxSide = max(width, height)
        if (initialMaxSide > MAX_SIDE_PX) {
            resolution *= initialMaxSide / MAX_SIDE_PX.toFloat()
            width = ceil((maxX - minX) / resolution).toInt() + PADDING_CELLS * 2
            height = ceil((maxY - minY) / resolution).toInt() + PADDING_CELLS * 2
        }
        if (width <= 0 || height <= 0 ||
            width.toLong() * height.toLong() > MAX_SIDE_PX.toLong() * MAX_SIDE_PX) {
            return null
        }

        val gridMinX = minX - PADDING_CELLS * resolution
        val gridMaxY = maxY + PADDING_CELLS * resolution
        val cellCount = width * height
        val occupiedScore = FloatArray(cellCount)
        val strongestOccupiedContribution = FloatArray(cellCount)
        val freeScore = FloatArray(cellCount)
        val latestSubmapByTrajectory = valid
            .groupBy { it.trajectoryId }
            .mapValues { (_, trajectoryTextures) ->
                trajectoryTextures.maxOf { it.submapIndex }
            }

        for (texture in valid) {
            val latestIndex = latestSubmapByTrajectory[texture.trajectoryId]
                ?: texture.submapIndex
            // The newest two submaps are normally still receiving scans and
            // should be slightly less authoritative during live rendering.
            // Once the trajectory has been finalized every submap is stable;
            // keeping the live penalty here weakened the final room visited by
            // the operator and could make its walls fail the fusion threshold.
            val submapWeight = if (finalized) {
                1.0f
            } else if (texture.submapIndex <= latestIndex - 2) {
                1.15f
            } else {
                0.85f
            }
            projectTexture(
                texture = texture,
                targetResolution = resolution,
                targetMinX = gridMinX,
                targetMaxY = gridMaxY,
                targetWidth = width,
                targetHeight = height,
                submapWeight = submapWeight,
                occupiedScore = occupiedScore,
                strongestOccupiedContribution = strongestOccupiedContribution,
                freeScore = freeScore
            )
        }

        val stableWall = BooleanArray(cellCount)
        for (y in 0 until height) {
            for (x in 0 until width) {
                val index = y * width + x
                val occupied = occupiedScore[index]
                if (occupied > 0f) {
                    var neighborhood = occupied
                    for (dy in -1..1) {
                        val sampleY = y + dy
                        if (sampleY !in 0 until height) continue
                        for (dx in -1..1) {
                            if (dx == 0 && dy == 0) continue
                            val sampleX = x + dx
                            if (sampleX !in 0 until width) continue
                            val distanceWeight = if (dx == 0 || dy == 0) 0.16f else 0.08f
                            neighborhood += occupiedScore[sampleY * width + sampleX] *
                                distanceWeight
                        }
                    }
                    // A cell is shown once in the global canvas. Agreement
                    // between overlapping submaps increases brightness instead
                    // of drawing the same wall repeatedly with alpha blending.
                    val supportFromOtherSubmaps =
                        occupied - strongestOccupiedContribution[index]
                    val hasCrossSubmapAgreement = supportFromOtherSubmaps >= 0.16f &&
                        neighborhood >= 0.86f
                    val isStrongContinuousSingleSubmapWall =
                        occupied >= 0.52f && neighborhood >= 0.94f
                    if (hasCrossSubmapAgreement ||
                        isStrongContinuousSingleSubmapWall) {
                        stableWall[index] = true
                    }
                }
            }
        }
        val structuralWall = if (finalized) {
            // Keep the gap-closed probability ridge for geometry extraction.
            // It is deliberately not skeletonized here: the native fitter
            // needs the full connected evidence around weak partitions.
            smoothFinalWalls(stableWall, width, height)
        } else {
            stableWall
        }
        val displayWall = if (finalized) {
            // Closing tiny gaps first keeps long walls connected. Thinning the
            // result afterwards produces the one-cell boundary used by mature
            // occupancy-map products instead of displaying a coarse blob.
            thinFinalWalls(
                structuralWall,
                width,
                height
            )
        } else {
            structuralWall
        }
        // Add verified long straight runs only to the finalized presentation.
        // The original fused centerline remains present underneath, and this
        // fitted overlay is never exported as algorithm evidence.
        val fittedDisplayWall = if (finalized) {
            fitLongWallSegmentsForDisplay(
                displayWall,
                width,
                height,
                resolution
            )
        } else {
            BooleanArray(cellCount)
        }
        val presentationWall = if (finalized) {
            BooleanArray(cellCount) { index ->
                displayWall[index] || fittedDisplayWall[index]
            }
        } else {
            displayWall
        }
        // One solid display pixel on each side makes the finalized wall easy
        // to read on a phone without changing the fused/exported wall mask.
        val boldPresentationWall = if (finalized) {
            expandDisplayWall(presentationWall, width, height, 1)
        } else {
            presentationWall
        }
        // A second, softer shoulder makes the increase obvious while the
        // original noisy/curved point-cloud silhouette remains recognizable.
        val widePresentationWall = if (finalized) {
            expandDisplayWall(presentationWall, width, height, 2)
        } else {
            presentationWall
        }
        val wallCoverage = smoothWallCoverage(
            widePresentationWall,
            width,
            height
        )

        val rawFreeSpace = BooleanArray(cellCount) { index ->
            freeScore[index] > 0.08f && !displayWall[index]
        }
        val renderedFreeSpace = if (finalized) {
            smoothFinalFreeSpace(rawFreeSpace, displayWall, width, height)
        } else {
            rawFreeSpace
        }

        val pixels = IntArray(cellCount)
        val visibleWall = BooleanArray(cellCount)
        // Preserve the original fused one-cell geometry. Final display only
        // strengthens its immediate antialiased shoulder, giving a modest
        // visual-width increase without dilation, vectorization or line fit.
        val presentationEdgeThreshold = if (finalized) 0.12f else 0.22f
        for (y in 0 until height) {
            for (x in 0 until width) {
                val index = y * width + x
                val occupied = occupiedScore[index]
                val coverage = wallCoverage[index]
                if (widePresentationWall[index] ||
                    coverage >= presentationEdgeThreshold) {
                    // The export mask follows the connected one-cell core.
                    // Fractional coverage below is presentation-only antialiasing.
                    if (displayWall[index]) visibleWall[index] = true
                    val confidence = (occupied / 1.70f).coerceIn(0f, 1f)
                    // Coverage supplies an anti-aliased edge around the
                    // probability ridge. Core wall cells stay opaque while
                    // fractional edge cells hide raster stair stepping.
                    val alpha = if (presentationWall[index]) {
                        if (fittedDisplayWall[index]) {
                            (218f + coverage * 20f)
                                .toInt().coerceIn(218, 238)
                        } else {
                            ((if (finalized) 195f else 165f) +
                                coverage * 34f + confidence * 18f)
                                .toInt().coerceIn(165, 235)
                        }
                    } else if (boldPresentationWall[index]) {
                        (145f + coverage * 45f)
                            .toInt().coerceIn(150, 188)
                    } else if (widePresentationWall[index]) {
                        (88f + coverage * 42f)
                            .toInt().coerceIn(96, 136)
                    } else {
                        if (finalized) {
                            (coverage * 220f).toInt().coerceIn(28, 75)
                        } else {
                            (coverage * 105f).toInt().coerceIn(18, 55)
                        }
                    }
                    val red = (185 + confidence * 30f).toInt().coerceAtMost(225)
                    val green = (232 + confidence * 23f).toInt().coerceAtMost(255)
                    val blue = (222 + confidence * 22f).toInt().coerceAtMost(248)
                    pixels[index] = Color.argb(alpha, red, green, blue)
                    continue
                }

                if (renderedFreeSpace[index]) {
                    val free = freeScore[index]
                    val alpha = (10 + min(1f, free) * 24f).toInt()
                    pixels[index] = Color.argb(alpha, 42, 62, 78)
                }
            }
        }

        val occupancyPixels = if (finalized) {
            suppressWeakExteriorRays(
                pixels = composeCartographerOccupancy(
                textures = valid,
                targetResolution = resolution,
                targetMinX = gridMinX,
                targetMaxY = gridMaxY,
                targetWidth = width,
                targetHeight = height
                ),
                visibleWall = visibleWall
            )
        } else {
            IntArray(cellCount) { 0xFF9A9A9A.toInt() }
        }

        return Result(
            bitmap = Bitmap.createBitmap(
                pixels,
                width,
                height,
                Bitmap.Config.ARGB_8888
            ),
            resolutionMetersPerPixel = resolution,
            worldMinX = gridMinX,
            worldMaxY = gridMaxY,
            contentMinX = minX,
            contentMaxX = maxX,
            contentMinY = minY,
            contentMaxY = maxY,
            // Keep Android's algorithm input identical to the offline
            // pbstream exporter: close the stable probability ridge first,
            // then reduce it to the same one-cell Zhang-Suen centerline.
            stableWallMask = displayWall,
            visibleWallMask = visibleWall,
            freeSpaceMask = renderedFreeSpace,
            occupancyPixels = occupancyPixels
        )
    }

    private fun selectLiveSubmaps(
        trajectory: List<SubmapTexture>
    ): List<SubmapTexture> {
        val sorted = trajectory.sortedBy { it.submapIndex }
        if (sorted.size <= MAX_LIVE_SUBMAPS_PER_TRAJECTORY) return sorted

        val recentStart = sorted.size - LIVE_RECENT_SUBMAPS_PER_TRAJECTORY
        val historySlots = MAX_LIVE_SUBMAPS_PER_TRAJECTORY -
            LIVE_RECENT_SUBMAPS_PER_TRAJECTORY
        val selected = ArrayList<SubmapTexture>(MAX_LIVE_SUBMAPS_PER_TRAJECTORY)
        for (slot in 0 until historySlots) {
            // Include both ends of the historical range so the complete mapped
            // extent remains visible while the amount of reprojection is fixed.
            val index = if (historySlots == 1) {
                0
            } else {
                (slot * (recentStart - 1).toFloat() / (historySlots - 1))
                    .roundToInt()
            }
            selected += sorted[index]
        }
        for (index in recentStart until sorted.size) selected += sorted[index]
        return selected.distinctBy { it.key }
    }

    /**
     * Cartographer intentionally records every miss update. On a neutral gray
     * occupancy canvas, a single weak miss chain is visible as a long gray ray
     * even though it is neither a wall nor a coherently explored floor area.
     * Hide only that low-confidence presentation band. Strong dark returns are
     * retained, so real point-cloud evidence outside the fitted outline is not
     * clipped. The structural masks and the pbstream are not changed here.
     */
    private fun suppressWeakExteriorRays(
        pixels: IntArray,
        visibleWall: BooleanArray
    ): IntArray {
        val unknown = 154
        for (index in pixels.indices) {
            if (visibleWall[index]) continue
            val value = Color.red(pixels[index])
            // Confident free cells are close to white and confident hits are
            // dark. Only the ambiguous middle band is visually normalized.
            if (value in 96..210) {
                pixels[index] = Color.rgb(unknown, unknown, unknown)
            }
        }
        return pixels
    }

    /**
     * Detects long, well-supported straight runs in the original fused
     * centerline. The result is an additive display overlay: it does not erase
     * unfitted pixels and is not returned as structural/algorithm evidence.
     */
    private fun fitLongWallSegmentsForDisplay(
        source: BooleanArray,
        width: Int,
        height: Int,
        resolutionMetersPerPixel: Float
    ): BooleanArray {
        var wallPointCount = 0
        for (occupied in source) if (occupied) wallPointCount++
        if (wallPointCount < 24) return BooleanArray(source.size)

        val wallPoints = IntArray(wallPointCount * 2)
        var pointOffset = 0
        for (y in 0 until height) {
            for (x in 0 until width) {
                if (!source[y * width + x]) continue
                wallPoints[pointOffset++] = x
                wallPoints[pointOffset++] = y
            }
        }

        val safeResolution = resolutionMetersPerPixel
            .takeIf { it.isFinite() && it > 1e-4f }
            ?: 0.05f
        val minimumLengthPixels = (0.90f / safeResolution)
            .roundToInt()
            .coerceIn(14, 60)
        // At 5 cm/px this bridges at most 3-4 pixels. A normal doorway remains
        // split into two fitted segments instead of becoming a closed wall.
        val maximumGapPixels = (0.18f / safeResolution)
            .roundToInt()
            .coerceIn(2, 5)
        val perpendicularTolerance = (0.075f / safeResolution)
            .coerceIn(1.0f, 2.0f)

        // Two-degree angular bins preserve genuine diagonal walls while still
        // regularizing scan-matching jitter along a long run.
        val angleStepDegrees = 2
        val angleCount = 180 / angleStepDegrees
        val centerX = (width - 1) * 0.5
        val centerY = (height - 1) * 0.5
        val maximumRho = ceil(hypot(width.toDouble(), height.toDouble()) * 0.5)
            .toInt() + 3
        val rhoCount = maximumRho * 2 + 1
        val cosines = DoubleArray(angleCount)
        val sines = DoubleArray(angleCount)
        for (angleIndex in 0 until angleCount) {
            val radians = angleIndex * angleStepDegrees * PI / 180.0
            cosines[angleIndex] = cos(radians)
            sines[angleIndex] = sin(radians)
        }

        val votes = IntArray(angleCount * rhoCount)
        val sampleStep = ceil(wallPointCount / 12_000.0)
            .toInt()
            .coerceAtLeast(1)
        var pointIndex = 0
        while (pointIndex < wallPointCount) {
            val x = wallPoints[pointIndex * 2] - centerX
            val y = wallPoints[pointIndex * 2 + 1] - centerY
            for (angleIndex in 0 until angleCount) {
                val rho = (x * cosines[angleIndex] +
                    y * sines[angleIndex]).roundToInt() + maximumRho
                if (rho in 0 until rhoCount) {
                    votes[angleIndex * rhoCount + rho]++
                }
            }
            pointIndex += sampleStep
        }

        data class HoughPeak(
            val angleIndex: Int,
            val rho: Int,
            val votes: Int
        )

        val minimumVotes = max(
            6,
            (minimumLengthPixels * 0.35f / sampleStep).roundToInt()
        )
        val peaks = ArrayList<HoughPeak>()
        for (angleIndex in 0 until angleCount) {
            val row = angleIndex * rhoCount
            for (rhoIndex in 1 until rhoCount - 1) {
                val vote = votes[row + rhoIndex]
                if (vote < minimumVotes ||
                    vote < votes[row + rhoIndex - 1] ||
                    vote < votes[row + rhoIndex + 1]) {
                    continue
                }
                peaks += HoughPeak(
                    angleIndex,
                    rhoIndex - maximumRho,
                    vote
                )
            }
        }
        peaks.sortByDescending { it.votes }

        val selectedPeaks = ArrayList<HoughPeak>()
        for (peak in peaks) {
            val duplicate = selectedPeaks.any { selected ->
                val rawAngleDistance = abs(
                    peak.angleIndex - selected.angleIndex
                )
                val angleDistance = min(
                    rawAngleDistance,
                    angleCount - rawAngleDistance
                )
                angleDistance <= 2 && abs(peak.rho - selected.rho) <= 3
            }
            if (!duplicate) selectedPeaks += peak
            if (selectedPeaks.size >= 48) break
        }

        val fitted = BooleanArray(source.size)
        val projections = IntArray(wallPointCount)
        var acceptedSegments = 0
        for (peak in selectedPeaks) {
            val normalX = cosines[peak.angleIndex]
            val normalY = sines[peak.angleIndex]
            val directionX = -normalY
            val directionY = normalX
            var projectionCount = 0
            for (index in 0 until wallPointCount) {
                val x = wallPoints[index * 2] - centerX
                val y = wallPoints[index * 2 + 1] - centerY
                val distance = abs(x * normalX + y * normalY - peak.rho)
                if (distance <= perpendicularTolerance) {
                    projections[projectionCount++] =
                        (x * directionX + y * directionY).roundToInt()
                }
            }
            if (projectionCount < minimumLengthPixels / 3) continue
            java.util.Arrays.sort(projections, 0, projectionCount)

            var runStart = projections[0]
            var previous = projections[0]
            var distinctSupport = 1
            fun flushRun(runEnd: Int) {
                val length = runEnd - runStart
                val supportRatio = distinctSupport /
                    max(1.0, length + 1.0)
                if (length < minimumLengthPixels || supportRatio < 0.38) return
                val firstX = centerX + peak.rho * normalX +
                    runStart * directionX
                val firstY = centerY + peak.rho * normalY +
                    runStart * directionY
                val secondX = centerX + peak.rho * normalX +
                    runEnd * directionX
                val secondY = centerY + peak.rho * normalY +
                    runEnd * directionY
                drawRasterLine(
                    fitted,
                    width,
                    height,
                    firstX.roundToInt(),
                    firstY.roundToInt(),
                    secondX.roundToInt(),
                    secondY.roundToInt()
                )
                acceptedSegments++
            }

            for (index in 1 until projectionCount) {
                val projection = projections[index]
                if (projection == previous) continue
                if (projection - previous > maximumGapPixels) {
                    flushRun(previous)
                    runStart = projection
                    distinctSupport = 1
                } else {
                    distinctSupport++
                }
                previous = projection
            }
            flushRun(previous)
            if (acceptedSegments >= 64) break
        }
        return fitted
    }

    private fun drawRasterLine(
        target: BooleanArray,
        width: Int,
        height: Int,
        startX: Int,
        startY: Int,
        endX: Int,
        endY: Int
    ) {
        var x = startX
        var y = startY
        val dx = abs(endX - startX)
        val stepX = if (startX < endX) 1 else -1
        val dy = -abs(endY - startY)
        val stepY = if (startY < endY) 1 else -1
        var error = dx + dy
        while (true) {
            if (x in 0 until width && y in 0 until height) {
                target[y * width + x] = true
            }
            if (x == endX && y == endY) break
            val doubledError = error * 2
            if (doubledError >= dy) {
                error += dy
                x += stepX
            }
            if (doubledError <= dx) {
                error += dx
                y += stepY
            }
        }
    }

    private fun expandDisplayWall(
        source: BooleanArray,
        width: Int,
        height: Int,
        radius: Int
    ): BooleanArray {
        val expanded = source.copyOf()
        for (y in 0 until height) {
            for (x in 0 until width) {
                if (!source[y * width + x]) continue
                for (dy in -radius..radius) {
                    val targetY = y + dy
                    if (targetY !in 0 until height) continue
                    for (dx in -radius..radius) {
                        // A rounded shoulder avoids square blocks at line
                        // endings and keeps diagonal scan contours natural.
                        if (dx * dx + dy * dy > radius * radius + 1) continue
                        val targetX = x + dx
                        if (targetX !in 0 until width) continue
                        expanded[targetY * width + targetX] = true
                    }
                }
            }
        }
        return expanded
    }

    /**
     * Removes one-cell free-space holes and isolated ray fragments without
     * expanding through occupied cells. Two local passes are enough to make
     * room faces visually coherent while preserving real door openings.
     */
    private fun smoothFinalFreeSpace(
        source: BooleanArray,
        wall: BooleanArray,
        width: Int,
        height: Int
    ): BooleanArray {
        var current = source.copyOf()
        repeat(2) {
            val next = current.copyOf()
            for (y in 1 until height - 1) {
                for (x in 1 until width - 1) {
                    val index = y * width + x
                    if (wall[index]) {
                        next[index] = false
                        continue
                    }
                    var neighbors = 0
                    for (dy in -1..1) {
                        for (dx in -1..1) {
                            if (dx == 0 && dy == 0) continue
                            if (current[(y + dy) * width + x + dx]) neighbors++
                        }
                    }
                    next[index] = if (current[index]) {
                        neighbors >= 2
                    } else {
                        neighbors >= 5
                    }
                }
            }
            current = next
        }
        for (index in current.indices) {
            if (wall[index]) current[index] = false
        }
        return current
    }

    /**
     * Zhang-Suen thinning reduces a multi-cell probability ridge to a
     * connected one-cell centerline. Junctions and line endings are retained,
     * unlike erosion which would shorten walls and open corners.
     */
    private fun thinFinalWalls(
        source: BooleanArray,
        width: Int,
        height: Int
    ): BooleanArray {
        val result = source.copyOf()
        val removalIndices = IntArray(result.size)
        var iteration = 0
        var changed: Boolean
        do {
            changed = false
            for (step in 0..1) {
                var removalCount = 0
                for (y in 1 until height - 1) {
                    for (x in 1 until width - 1) {
                        val index = y * width + x
                        if (!result[index]) continue

                        val p2 = result[index - width]
                        val p3 = result[index - width + 1]
                        val p4 = result[index + 1]
                        val p5 = result[index + width + 1]
                        val p6 = result[index + width]
                        val p7 = result[index + width - 1]
                        val p8 = result[index - 1]
                        val p9 = result[index - width - 1]
                        var neighborCount = 0
                        if (p2) neighborCount++
                        if (p3) neighborCount++
                        if (p4) neighborCount++
                        if (p5) neighborCount++
                        if (p6) neighborCount++
                        if (p7) neighborCount++
                        if (p8) neighborCount++
                        if (p9) neighborCount++
                        if (neighborCount !in 2..6) continue

                        var transitions = 0
                        if (!p2 && p3) transitions++
                        if (!p3 && p4) transitions++
                        if (!p4 && p5) transitions++
                        if (!p5 && p6) transitions++
                        if (!p6 && p7) transitions++
                        if (!p7 && p8) transitions++
                        if (!p8 && p9) transitions++
                        if (!p9 && p2) transitions++
                        if (transitions != 1) continue

                        val keepCornerA: Boolean
                        val keepCornerB: Boolean
                        if (step == 0) {
                            keepCornerA = p2 && p4 && p6
                            keepCornerB = p4 && p6 && p8
                        } else {
                            keepCornerA = p2 && p4 && p8
                            keepCornerB = p2 && p6 && p8
                        }
                        if (!keepCornerA && !keepCornerB) {
                            removalIndices[removalCount++] = index
                        }
                    }
                }
                for (i in 0 until removalCount) {
                    result[removalIndices[i]] = false
                }
                if (removalCount > 0) changed = true
            }
            iteration++
        } while (changed && iteration < 16)
        return result
    }

    private fun smoothWallCoverage(
        wall: BooleanArray,
        width: Int,
        height: Int
    ): FloatArray {
        val coverage = FloatArray(wall.size)
        for (y in 1 until height - 1) {
            for (x in 1 until width - 1) {
                val index = y * width + x
                var weighted = if (wall[index]) 4f else 0f
                if (wall[index - 1]) weighted += 2f
                if (wall[index + 1]) weighted += 2f
                if (wall[index - width]) weighted += 2f
                if (wall[index + width]) weighted += 2f
                if (wall[index - width - 1]) weighted += 1f
                if (wall[index - width + 1]) weighted += 1f
                if (wall[index + width - 1]) weighted += 1f
                if (wall[index + width + 1]) weighted += 1f
                coverage[index] = weighted / 16f
            }
        }
        return coverage
    }

    /**
     * A conservative raster cleanup used only after final optimization.
     * It removes isolated returns and joins one-cell wall gaps, but never
     * vectorizes the map or replaces it with the fitted floor-plan outline.
     */
    private fun smoothFinalWalls(
        source: BooleanArray,
        width: Int,
        height: Int
    ): BooleanArray {
        var current = source.copyOf()
        repeat(2) {
            val next = current.copyOf()
            for (y in 1 until height - 1) {
                for (x in 1 until width - 1) {
                    val index = y * width + x
                    var neighbors = 0
                    for (dy in -1..1) {
                        for (dx in -1..1) {
                            if (dx == 0 && dy == 0) continue
                            if (current[(y + dy) * width + x + dx]) neighbors++
                        }
                    }
                    if (current[index]) {
                        // Remove isolated speckles while retaining line ends.
                        next[index] = neighbors >= 1
                    } else {
                        // Close only a single-cell gap with evidence on both
                        // sides. This does not fill rooms or close doorways.
                        val horizontal = current[index - 1] && current[index + 1]
                        val vertical = current[index - width] && current[index + width]
                        val diagonalA = current[index - width - 1] &&
                            current[index + width + 1]
                        val diagonalB = current[index - width + 1] &&
                            current[index + width - 1]
                        next[index] = horizontal || vertical || diagonalA || diagonalB
                    }
                }
            }
            current = next
        }
        return current
    }

    private fun projectTexture(
        texture: SubmapTexture,
        targetResolution: Float,
        targetMinX: Float,
        targetMaxY: Float,
        targetWidth: Int,
        targetHeight: Int,
        submapWeight: Float,
        occupiedScore: FloatArray,
        strongestOccupiedContribution: FloatArray,
        freeScore: FloatArray
    ) {
        val cosTheta = cos(texture.theta).toFloat()
        val sinTheta = sin(texture.theta).toFloat()
        val sourceStepXWorldX = sinTheta * texture.resolution
        val sourceStepXWorldY = -cosTheta * texture.resolution
        val sourceStepYWorldX = -cosTheta * texture.resolution
        val sourceStepYWorldY = -sinTheta * texture.resolution
        val halfCellWorldX = (sourceStepXWorldX + sourceStepYWorldX) * 0.5f
        val halfCellWorldY = (sourceStepXWorldY + sourceStepYWorldY) * 0.5f

        for (py in 0 until texture.height) {
            var worldX = texture.originX + sourceStepYWorldX * py + halfCellWorldX
            var worldY = texture.originY + sourceStepYWorldY * py + halfCellWorldY
            val rowOffset = py * texture.width
            for (px in 0 until texture.width) {
                val rawCell = texture.rawCells.getOrNull(rowOffset + px) ?: 0
                if (rawCell != 0) {
                    // Structural fusion must consume Cartographer's untouched
                    // probability texture. The highlighted ARGB plane applies
                    // an extra single-submap neighborhood filter for live-map
                    // presentation; using that plane here made Android discard
                    // weak walls before cross-submap agreement could recover
                    // them and diverged from export_pbstream_floorplan.py.
                    val probability = decodeTextureProbability(rawCell)
                    val wallAlpha = occupiedAlpha(probability)
                    val missAlpha = freeAlpha(probability)
                    val targetXFloat = (worldX - targetMinX) / targetResolution
                    val targetYFloat = (targetMaxY - worldY) / targetResolution
                    val baseX = floor(targetXFloat).toInt()
                    val baseY = floor(targetYFloat).toInt()
                    val fractionX = targetXFloat - baseX
                    val fractionY = targetYFloat - baseY
                    // Bilinear splatting preserves the optimized submap's
                    // sub-pixel pose. Integer truncation made gently rotated
                    // walls look like coarse stair steps.
                    for (offsetY in 0..1) {
                        val targetY = baseY + offsetY
                        if (targetY !in 0 until targetHeight) continue
                        val weightY = if (offsetY == 0) 1f - fractionY else fractionY
                        for (offsetX in 0..1) {
                            val targetX = baseX + offsetX
                            if (targetX !in 0 until targetWidth) continue
                            val weightX = if (offsetX == 0) 1f - fractionX else fractionX
                            val interpolationWeight = weightX * weightY
                            if (interpolationWeight <= 0.001f) continue
                            val targetIndex = targetY * targetWidth + targetX
                            if (wallAlpha > 0f) {
                                val contribution = submapWeight *
                                    wallAlpha * interpolationWeight
                                occupiedScore[targetIndex] += contribution
                                strongestOccupiedContribution[targetIndex] = max(
                                    strongestOccupiedContribution[targetIndex],
                                    contribution
                                )
                            } else if (missAlpha > 0f) {
                                freeScore[targetIndex] += submapWeight * 0.30f *
                                    (missAlpha / FREE_MAX_ALPHA.toFloat()) *
                                    interpolationWeight
                            }
                        }
                    }
                }
                worldX += sourceStepXWorldX
                worldY += sourceStepXWorldY
            }
        }
    }

    /** Inverts ProbabilityGrid::DrawToSubmapTexture's log-odds encoding. */
    private fun decodeTextureProbability(rawCell: Int): Float {
        val intensity = (rawCell ushr 8) and 0xFF
        val alpha = rawCell and 0xFF
        val logOddsInteger = when {
            intensity > 0 -> 128 - intensity
            alpha > 1 -> 128 + alpha
            else -> 128
        }.coerceIn(1, 255)
        val normalized = (logOddsInteger - 1) / 254f
        val logOdds = MIN_LOG_ODDS +
            normalized * (MAX_LOG_ODDS - MIN_LOG_ODDS)
        return (1.0 / (1.0 + exp(-logOdds.toDouble())))
            .toFloat()
            .coerceIn(MIN_PROBABILITY, MAX_PROBABILITY)
    }

    /** Matches tools/export_pbstream_floorplan.py occupied_alpha(). */
    private fun occupiedAlpha(probability: Float): Float = when {
        probability < MIN_OCCUPIED_PROBABILITY -> 0f
        probability < RELIABLE_OCCUPIED_PROBABILITY ->
            (90f + (probability - MIN_OCCUPIED_PROBABILITY) /
                (RELIABLE_OCCUPIED_PROBABILITY - MIN_OCCUPIED_PROBABILITY) *
                100f) / 255f
        else ->
            (200f + (probability - RELIABLE_OCCUPIED_PROBABILITY) /
                (MAX_PROBABILITY - RELIABLE_OCCUPIED_PROBABILITY) * 55f) /
                255f
    }

    /** Matches tools/export_pbstream_floorplan.py free_alpha(). */
    private fun freeAlpha(probability: Float): Float {
        if (probability >= 0.5f) return 0f
        return 12f + (0.5f - probability) /
            (0.5f - MIN_PROBABILITY) * 30f
    }

    /**
     * Reproduces Cartographer's PaintSubmapSlices channel composition for the
     * finalized neutral occupancy image. Each optimized submap is inverse
     * sampled at target-pixel centers, then its premultiplied intensity/alpha
     * channels are painted with SOURCE_OVER semantics. This is intentionally
     * separate from the thresholded highlighted-map masks above: averaging
     * probabilities exposed individual scan rays as gray bands, while fitting
     * polygons changed the measured map geometry.
     */
    private fun composeCartographerOccupancy(
        textures: List<SubmapTexture>,
        targetResolution: Float,
        targetMinX: Float,
        targetMaxY: Float,
        targetWidth: Int,
        targetHeight: Int
    ): IntArray {
        val count = targetWidth * targetHeight
        // PaintSubmapSlices starts with red=128 and observed=0. We store only
        // the two channels consumed when converting its Cairo surface to an
        // occupancy image.
        val color = FloatArray(count) { 128f }
        val observed = FloatArray(count)

        for (texture in textures.sortedWith(
            compareBy<SubmapTexture> { it.trajectoryId }.thenBy { it.submapIndex }
        )) {
            if (texture.rawCells.size < texture.width * texture.height) continue
            val cosTheta = cos(texture.theta).toFloat()
            val sinTheta = sin(texture.theta).toFloat()

            val corners = arrayOf(
                texturePixelToWorld(texture, 0f, 0f),
                texturePixelToWorld(texture, texture.width.toFloat(), 0f),
                texturePixelToWorld(texture, 0f, texture.height.toFloat()),
                texturePixelToWorld(
                    texture,
                    texture.width.toFloat(),
                    texture.height.toFloat()
                )
            )
            var minTargetX = targetWidth - 1
            var maxTargetX = 0
            var minTargetY = targetHeight - 1
            var maxTargetY = 0
            for ((worldX, worldY) in corners) {
                val targetX = ((worldX - targetMinX) / targetResolution).toInt()
                val targetY = ((targetMaxY - worldY) / targetResolution).toInt()
                minTargetX = min(minTargetX, targetX)
                maxTargetX = max(maxTargetX, targetX)
                minTargetY = min(minTargetY, targetY)
                maxTargetY = max(maxTargetY, targetY)
            }
            minTargetX = (minTargetX - 2).coerceIn(0, targetWidth - 1)
            maxTargetX = (maxTargetX + 2).coerceIn(0, targetWidth - 1)
            minTargetY = (minTargetY - 2).coerceIn(0, targetHeight - 1)
            maxTargetY = (maxTargetY + 2).coerceIn(0, targetHeight - 1)

            for (targetY in minTargetY..maxTargetY) {
                val worldY = targetMaxY - (targetY + 0.5f) * targetResolution
                for (targetX in minTargetX..maxTargetX) {
                    val worldX = targetMinX + (targetX + 0.5f) * targetResolution
                    val deltaX = worldX - texture.originX
                    val deltaY = worldY - texture.originY
                    val sourceX = (sinTheta * deltaX - cosTheta * deltaY) /
                        texture.resolution - 0.5f
                    val sourceY = (-cosTheta * deltaX - sinTheta * deltaY) /
                        texture.resolution - 0.5f
                    val baseX = floor(sourceX).toInt()
                    val baseY = floor(sourceY).toInt()
                    val fractionX = sourceX - baseX
                    val fractionY = sourceY - baseY

                    var sampledIntensity = 0f
                    var sampledAlpha = 0f
                    var sampledObserved = 0f
                    for (offsetY in 0..1) {
                        val sourcePixelY = baseY + offsetY
                        if (sourcePixelY !in 0 until texture.height) continue
                        val weightY = if (offsetY == 0) 1f - fractionY else fractionY
                        for (offsetX in 0..1) {
                            val sourcePixelX = baseX + offsetX
                            if (sourcePixelX !in 0 until texture.width) continue
                            val weightX = if (offsetX == 0) 1f - fractionX else fractionX
                            val weight = weightX * weightY
                            if (weight <= 0f) continue
                            val rawCell = texture.rawCells[
                                sourcePixelY * texture.width + sourcePixelX
                            ]
                            if (rawCell == 0) continue
                            sampledIntensity += ((rawCell ushr 8) and 0xFF) * weight
                            sampledAlpha += (rawCell and 0xFF) * weight
                            sampledObserved += 255f * weight
                        }
                    }
                    if (sampledObserved < 0.5f) continue

                    val targetIndex = targetY * targetWidth + targetX
                    val inverseAlpha = 1f - (sampledAlpha / 255f).coerceIn(0f, 1f)
                    color[targetIndex] = (
                        sampledIntensity + color[targetIndex] * inverseAlpha
                    ).coerceIn(0f, 255f)
                    observed[targetIndex] = (
                        sampledObserved + observed[targetIndex] * inverseAlpha
                    ).coerceIn(0f, 255f)
                }
            }
        }

        return IntArray(count) { index ->
            val value = if (observed[index] >= 0.5f) {
                color[index].roundToInt().coerceIn(0, 255)
            } else {
                154
            }
            Color.rgb(value, value, value)
        }
    }

    private fun texturePixelToWorld(
        texture: SubmapTexture,
        px: Float,
        py: Float
    ): Pair<Float, Float> {
        val cosTheta = cos(texture.theta).toFloat()
        val sinTheta = sin(texture.theta).toFloat()
        val localX = px * texture.resolution
        val localY = py * texture.resolution
        return Pair(
            texture.originX + sinTheta * localX - cosTheta * localY,
            texture.originY - cosTheta * localX - sinTheta * localY
        )
    }
}
