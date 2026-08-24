package com.cartographer.demo

import android.content.Context
import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Matrix
import android.graphics.Paint
import android.graphics.Path
import android.graphics.PointF
import android.graphics.RectF
import android.util.AttributeSet
import android.util.TypedValue
import android.os.SystemClock
import android.view.MotionEvent
import android.view.ScaleGestureDetector
import android.view.View
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import java.util.Locale
import java.util.concurrent.Executors

class LidarMapView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null
) : View(context, attrs) {

    private enum class OrientationMode {
        FOLLOW_ROBOT,
        MAGNETIC_NORTH_OVERVIEW
    }

    private val trajectory = ArrayDeque<PointF>()
    private val optimizedTrajectory = ArrayList<PointF>()
    private val liveScanPoints = ArrayList<PointF>(720)
    private var liveScanTimestampNs = 0L
    private var liveScanVisible = true
    private val trajectoryPath = Path()
    private var latestPose = Pose2D(0.0, 0.0, 0.0)
    private var orientationMode = OrientationMode.FOLLOW_ROBOT
    private var overviewCenterX = 0f
    private var overviewCenterY = 0f
    private var magneticNorthYawRadians = 0f
    private var mapMeasurement: MapMeasurement? = null
    private val submaps = LinkedHashMap<String, SubmapTexture>()
    private val liveSubmapOverlays = LinkedHashMap<String, LiveSubmapOverlay>()
    private val fusionExecutor = Executors.newSingleThreadExecutor { runnable ->
        Thread(runnable, "FusedMapRenderer").apply { isDaemon = true }
    }
    private var fusedMap: FusedMapRenderer.Result? = null
    private var floorPlanOverlayBitmap: Bitmap? = null
    private var heatMapOverlayBitmap: Bitmap? = null
    private var pointCloudVisible = true
    private var floorPlanOverlayVisible = true
    private var heatMapOverlayVisible = true
    private var showingFinalizedOccupancy = false
    private var fusionGeneration = 0L
    private var fusionRunning = false
    private var fusionPending = false
    private var fusionThrottleScheduled = false
    private var lastFusionStartMs = 0L
    private val fusionThrottleRunnable = Runnable {
        fusionThrottleScheduled = false
        startNextFusedMapRender()
    }
    private val submapMatrix = Matrix()
    private val submapMatrixValues = FloatArray(9)
    private val scratchPoint = PointF()
    private var userZoom = 1f
    private var panOffsetX = 0f
    private var panOffsetY = 0f
    private var lastTouchX = 0f
    private var lastTouchY = 0f
    private var isDragging = false
    private val scaleGestureDetector = ScaleGestureDetector(
        context,
        object : ScaleGestureDetector.SimpleOnScaleGestureListener() {
            override fun onScale(detector: ScaleGestureDetector): Boolean {
                userZoom = (userZoom * detector.scaleFactor).coerceIn(MIN_USER_ZOOM, MAX_USER_ZOOM)
                invalidate()
                return true
            }

            override fun onScaleBegin(detector: ScaleGestureDetector): Boolean {
                parent?.requestDisallowInterceptTouchEvent(true)
                return true
            }
        }
    )

    private val backgroundPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(18, 22, 28)
        style = Paint.Style.FILL
    }
    private val gridPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(48, 56, 66)
        strokeWidth = 1f
    }
    private val submapPaint = Paint(Paint.FILTER_BITMAP_FLAG).apply {
        alpha = 255
    }
    private val heatMapPaint = Paint(Paint.FILTER_BITMAP_FLAG).apply {
        alpha = 190
    }
    private val floorPlanOverlayPaint = Paint(Paint.FILTER_BITMAP_FLAG).apply {
        alpha = 255
    }
    private val liveScanPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(100, 235, 255)
        alpha = 190
        strokeWidth = 3f
        strokeCap = Paint.Cap.ROUND
    }
    private val pathPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(255, 203, 92)
        strokeWidth = 4f
        strokeCap = Paint.Cap.ROUND
        strokeJoin = Paint.Join.ROUND
        style = Paint.Style.STROKE
    }
    private val robotPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE
        style = Paint.Style.FILL
    }
    private val measurementPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(255, 214, 90)
        strokeWidth = 2.5f
        strokeCap = Paint.Cap.ROUND
        strokeJoin = Paint.Join.ROUND
        style = Paint.Style.STROKE
    }
    private val measurementGuidePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.argb(150, 255, 214, 90)
        strokeWidth = 1.5f
        style = Paint.Style.STROKE
    }
    private val measurementTextPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE
        textSize = TypedValue.applyDimension(
            TypedValue.COMPLEX_UNIT_SP,
            13f,
            resources.displayMetrics
        )
        textAlign = Paint.Align.CENTER
        style = Paint.Style.FILL
    }
    private val measurementTextBackgroundPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.argb(210, 28, 33, 40)
        style = Paint.Style.FILL
    }

    fun setLiveScan(scan: LidarScan, pose: Pose2D?) {
        if (scan.timestampNs <= liveScanTimestampNs) return
        liveScanTimestampNs = scan.timestampNs
        val p = pose ?: latestPose
        latestPose = p

        val poseX = p.x.toFloat()
        val poseY = p.y.toFloat()
        val pathPoint = PointF(poseX, poseY)
        val lastPathPoint = trajectory.lastOrNull()
        if (lastPathPoint == null ||
            squaredDistance(lastPathPoint, pathPoint) > 0.0025f) {
            trajectory.addLast(pathPoint)
            while (trajectory.size > 2000) trajectory.removeFirst()
        }

        liveScanPoints.clear()
        val count = minOf(scan.ranges.size, scan.anglesDeg.size)
        for (i in 0 until count) {
            val range = scan.ranges[i]
            if (!range.isFinite() || range < 0.05f || range > 8.0f) continue
            val angle = Math.toRadians(scan.anglesDeg[i].toDouble())
            liveScanPoints.add(
                PointF(
                    (range * cos(angle)).toFloat(),
                    (range * sin(angle)).toFloat()
                )
            )
        }

        invalidate()
    }

    fun clearLiveScan() {
        if (liveScanPoints.isEmpty() && liveScanTimestampNs == 0L) return
        liveScanPoints.clear()
        liveScanTimestampNs = 0L
        invalidate()
    }

    fun setLiveScanVisible(visible: Boolean) {
        liveScanVisible = visible
        if (!visible) {
            liveScanPoints.clear()
            liveScanTimestampNs = 0L
        }
        invalidate()
    }

    fun setSubmapTexture(texture: SubmapTexture?) {
        if (texture == null) return
        setSubmapTextures(listOf(texture))
    }

    fun setSubmapTextures(textures: List<SubmapTexture>) {
        val wasShowingFinalizedOccupancy = showingFinalizedOccupancy
        showingFinalizedOccupancy = false
        var changed = false
        val changedTextures = ArrayList<SubmapTexture>(textures.size)
        for (texture in textures) {
            val current = submaps[texture.key]
            if (current != null && current.version == texture.version &&
                current.samePoseAs(texture)) {
                continue
            }
            submaps[texture.key] = texture
            changedTextures += texture
            changed = true
        }
        if (!changed) {
            // Continuing a saved map can initially return the same submap
            // versions. Re-render them in the live highlight style instead of
            // leaving the finalized black occupancy raster on screen.
            if (wasShowingFinalizedOccupancy && submaps.isNotEmpty()) {
                requestFusedMapRender()
            }
            invalidate()
            return
        }
        // A periodic pose-graph refresh can move many historical submaps at
        // once. Only the newest two can still receive scan data, so limiting
        // the immediate layer avoids allocating dozens of Bitmaps on the UI
        // thread while the background fusion already handles the full list.
        val newestTrajectoryId = changedTextures.maxOf { it.trajectoryId }
        val liveTextures = changedTextures
            .asSequence()
            .filter { it.trajectoryId == newestTrajectoryId }
            .sortedByDescending { it.submapIndex }
            .take(2)
            .toList()
        val liveKeys = liveTextures.mapTo(HashSet()) { it.key }
        val staleOverlays = liveSubmapOverlays.entries.iterator()
        while (staleOverlays.hasNext()) {
            val entry = staleOverlays.next()
            if (entry.key !in liveKeys) {
                entry.value.bitmap.recycle()
                staleOverlays.remove()
            }
        }
        for (texture in liveTextures) {
            val bitmap = Bitmap.createBitmap(
                texture.pixels,
                texture.width,
                texture.height,
                Bitmap.Config.ARGB_8888
            )
            liveSubmapOverlays.put(
                texture.key,
                LiveSubmapOverlay(texture, bitmap)
            )?.bitmap?.recycle()
        }
        requestFusedMapRender()
        // The active texture is visible immediately. The more expensive global
        // confidence fusion continues on the renderer thread and replaces it.
        invalidate()
    }

    fun setFinalizedSubmapTextures(textures: List<SubmapTexture>) {
        if (textures.isEmpty()) return
        showingFinalizedOccupancy = true
        recycleLiveSubmapOverlays()
        fusionGeneration++
        fusionPending = false
        val generation = fusionGeneration
        val snapshot = textures.toList()
        fusionExecutor.execute {
            val rendered = try {
                FusedMapRenderer.render(snapshot, finalized = true)?.let { fused ->
                    try {
                        val occupancyBitmap = Bitmap.createBitmap(
                            fused.occupancyPixels,
                            fused.bitmap.width,
                            fused.bitmap.height,
                            Bitmap.Config.ARGB_8888
                        )
                        fused.bitmap.recycle()
                        fused.copy(bitmap = occupancyBitmap)
                    } catch (error: Throwable) {
                        fused.bitmap.recycle()
                        throw error
                    }
                }
            } catch (_: Throwable) {
                null
            }
            post {
                if (generation == fusionGeneration && rendered != null) {
                    val previous = fusedMap
                    fusedMap = rendered
                    if (previous?.bitmap !== rendered.bitmap) {
                        previous?.bitmap?.recycle()
                    }
                    invalidate()
                } else {
                    rendered?.bitmap?.recycle()
                    if (generation == fusionGeneration) {
                        showingFinalizedOccupancy = false
                        invalidate()
                    }
                }
            }
        }
    }

    /**
     * Replaces only the pixels of the finalized fused map. The bitmap is
     * generated in the same export geometry, so the existing world transform,
     * measurement overlay and gestures remain valid.
     */
    fun setFinalizedOccupancyBitmap(bitmap: Bitmap) {
        val current = fusedMap
        if (current == null ||
            bitmap.width != current.bitmap.width ||
            bitmap.height != current.bitmap.height) {
            bitmap.recycle()
            return
        }
        val previous = current.bitmap
        fusedMap = current.copy(bitmap = bitmap)
        showingFinalizedOccupancy = true
        if (previous !== bitmap && !previous.isRecycled) previous.recycle()
        invalidate()
    }

    /** Sets a transparent floor-plan layer in the fused-map pixel frame. */
    fun setFloorPlanOverlay(bitmap: Bitmap?): Boolean {
        val current = fusedMap
        if (bitmap != null && current != null && (
                bitmap.width != current.bitmap.width ||
                bitmap.height != current.bitmap.height)) {
            bitmap.recycle()
            return false
        }
        val previous = floorPlanOverlayBitmap
        floorPlanOverlayBitmap = bitmap
        if (previous !== bitmap && previous?.isRecycled == false) previous.recycle()
        invalidate()
        return true
    }

    /** Sets a transparent heat-map layer in the fused-map pixel frame. */
    fun setHeatMapOverlay(bitmap: Bitmap?): Boolean {
        val current = fusedMap
        if (bitmap != null && current != null && (
                bitmap.width != current.bitmap.width ||
                bitmap.height != current.bitmap.height)) {
            bitmap.recycle()
            return false
        }
        val previous = heatMapOverlayBitmap
        heatMapOverlayBitmap = bitmap
        if (previous !== bitmap && previous?.isRecycled == false) previous.recycle()
        invalidate()
        return true
    }

    fun setMapLayerVisibility(
        pointCloud: Boolean? = null,
        floorPlan: Boolean? = null,
        heatMap: Boolean? = null
    ) {
        pointCloud?.let { pointCloudVisible = it }
        floorPlan?.let { floorPlanOverlayVisible = it }
        heatMap?.let { heatMapOverlayVisible = it }
        invalidate()
    }

    fun clearMapOverlays() {
        floorPlanOverlayBitmap?.recycle()
        floorPlanOverlayBitmap = null
        heatMapOverlayBitmap?.recycle()
        heatMapOverlayBitmap = null
        invalidate()
    }

    fun setOptimizedTrajectory(points: List<PointF>) {
        optimizedTrajectory.clear()
        optimizedTrajectory.addAll(points)
        invalidate()
    }

    fun setMapMeasurement(measurement: MapMeasurement?) {
        mapMeasurement = measurement
        invalidate()
    }


    fun resetToRobotFollowing() {
        orientationMode = OrientationMode.FOLLOW_ROBOT
        panOffsetX = 0f
        panOffsetY = 0f
        invalidate()
    }

    fun showMagneticNorthOverview(
        measurement: MapMeasurement,
        northYawRadians: Float
    ): Boolean {
        if (width <= 0 || height <= 0 || measurement.xSizeMeters <= 0f ||
            measurement.ySizeMeters <= 0f || !northYawRadians.isFinite()) {
            return false
        }
        overviewCenterX = (measurement.minX + measurement.maxX) * 0.5f
        overviewCenterY = (measurement.minY + measurement.maxY) * 0.5f
        magneticNorthYawRadians = northYawRadians

        val cosRotation = cos(northYawRadians).toFloat()
        val sinRotation = sin(northYawRadians).toFloat()
        var minScreenX = Float.POSITIVE_INFINITY
        var maxScreenX = Float.NEGATIVE_INFINITY
        var minScreenY = Float.POSITIVE_INFINITY
        var maxScreenY = Float.NEGATIVE_INFINITY
        val corners = arrayOf(
            measurement.minX to measurement.minY,
            measurement.minX to measurement.maxY,
            measurement.maxX to measurement.minY,
            measurement.maxX to measurement.maxY
        )
        for ((worldX, worldY) in corners) {
            val x = worldX - overviewCenterX
            val y = -(worldY - overviewCenterY)
            val rotatedX = cosRotation * x - sinRotation * y
            val rotatedY = sinRotation * x + cosRotation * y
            minScreenX = min(minScreenX, rotatedX)
            maxScreenX = max(maxScreenX, rotatedX)
            minScreenY = min(minScreenY, rotatedY)
            maxScreenY = max(maxScreenY, rotatedY)
        }
        val spanX = (maxScreenX - minScreenX).coerceAtLeast(0.01f)
        val spanY = (maxScreenY - minScreenY).coerceAtLeast(0.01f)
        val padding = 58f * resources.displayMetrics.density
        val availableWidth = (width - padding * 2f).coerceAtLeast(1f)
        val availableHeight = (height - padding * 2f).coerceAtLeast(1f)
        val fittedScale = min(availableWidth / spanX, availableHeight / spanY)
        userZoom = (fittedScale / BASE_SCALE).coerceIn(MIN_USER_ZOOM, MAX_USER_ZOOM)
        panOffsetX = 0f
        panOffsetY = 0f
        orientationMode = OrientationMode.MAGNETIC_NORTH_OVERVIEW
        invalidate()
        return true
    }

    fun clearMap() {
        trajectory.clear()
        optimizedTrajectory.clear()
        liveScanPoints.clear()
        liveScanTimestampNs = 0L
        submaps.clear()
        recycleLiveSubmapOverlays()
        fusionGeneration++
        fusionPending = false
        fusedMap?.bitmap?.recycle()
        fusedMap = null
        floorPlanOverlayBitmap?.recycle()
        floorPlanOverlayBitmap = null
        heatMapOverlayBitmap?.recycle()
        heatMapOverlayBitmap = null
        showingFinalizedOccupancy = false
        mapMeasurement = null
        latestPose = Pose2D(0.0, 0.0, 0.0)
        orientationMode = OrientationMode.FOLLOW_ROBOT
        overviewCenterX = 0f
        overviewCenterY = 0f
        magneticNorthYawRadians = 0f
        userZoom = 1f
        panOffsetX = 0f
        panOffsetY = 0f
        invalidate()
    }

    override fun onTouchEvent(event: MotionEvent): Boolean {
        scaleGestureDetector.onTouchEvent(event)

        when (event.actionMasked) {
            MotionEvent.ACTION_DOWN -> {
                parent?.requestDisallowInterceptTouchEvent(true)
                lastTouchX = event.x
                lastTouchY = event.y
                isDragging = true
            }
            MotionEvent.ACTION_POINTER_DOWN -> {
                isDragging = false
            }
            MotionEvent.ACTION_MOVE -> {
                if (isDragging && event.pointerCount == 1 && !scaleGestureDetector.isInProgress) {
                    panOffsetX += event.x - lastTouchX
                    panOffsetY += event.y - lastTouchY
                    lastTouchX = event.x
                    lastTouchY = event.y
                    invalidate()
                }
            }
            MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                isDragging = false
                parent?.requestDisallowInterceptTouchEvent(false)
            }
        }
        return true
    }

    override fun onDetachedFromWindow() {
        fusionGeneration++
        fusionPending = false
        removeCallbacks(fusionThrottleRunnable)
        fusionThrottleScheduled = false
        fusionExecutor.shutdownNow()
        recycleLiveSubmapOverlays()
        fusedMap?.bitmap?.recycle()
        fusedMap = null
        floorPlanOverlayBitmap?.recycle()
        floorPlanOverlayBitmap = null
        heatMapOverlayBitmap?.recycle()
        heatMapOverlayBitmap = null
        super.onDetachedFromWindow()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        if (showingFinalizedOccupancy) {
            canvas.drawColor(FINALIZED_UNKNOWN_COLOR)
        } else {
            canvas.drawRect(0f, 0f, width.toFloat(), height.toFloat(), backgroundPaint)
        }

        val scale = BASE_SCALE * userZoom
        val centerX = width / 2f + panOffsetX
        val centerY = height / 2f + panOffsetY

        if (!showingFinalizedOccupancy) {
            drawGrid(canvas, centerX, centerY, scale)
        }
        canvas.save()
        canvas.scale(MAP_MIRROR_X, 1f, centerX, centerY)
        canvas.rotate(currentMapRotationDegrees(), centerX, centerY)
        drawSubmaps(canvas, centerX, centerY, scale)
        if (!showingFinalizedOccupancy) {
            drawLiveSubmapOverlays(canvas, centerX, centerY, scale)
            if (liveScanVisible) {
                drawLiveScan(canvas, centerX, centerY, scale)
            }

            val pathPoints: Iterable<PointF> =
                if (optimizedTrajectory.size > 1) optimizedTrajectory else trajectory
            val pathSize = (pathPoints as? Collection<PointF>)?.size ?: trajectory.size
            if (pathSize > 1) {
                val referenceX = currentReferenceX()
                val referenceY = currentReferenceY()
                trajectoryPath.reset()
                var first = true
                for (point in pathPoints) {
                    val x = centerX + (point.x - referenceX) * scale
                    val y = centerY - (point.y - referenceY) * scale
                    if (first) {
                        trajectoryPath.moveTo(x, y)
                        first = false
                    } else {
                        trajectoryPath.lineTo(x, y)
                    }
                }
                canvas.drawPath(trajectoryPath, pathPaint)
            }

            val robotX = centerX + (latestPose.x.toFloat() - currentReferenceX()) * scale
            val robotY = centerY - (latestPose.y.toFloat() - currentReferenceY()) * scale
            canvas.drawCircle(robotX, robotY, 7f, robotPaint)
            canvas.drawLine(
                robotX,
                robotY,
                robotX + cos(latestPose.theta).toFloat() * 22f,
                robotY - sin(latestPose.theta).toFloat() * 22f,
                robotPaint
            )
        }
        canvas.restore()
    }

    private fun currentMapRotationDegrees(): Float {
        return when (orientationMode) {
            OrientationMode.FOLLOW_ROBOT -> Math.toDegrees(latestPose.theta).toFloat() - 90f
            OrientationMode.MAGNETIC_NORTH_OVERVIEW ->
                Math.toDegrees(magneticNorthYawRadians.toDouble()).toFloat()
        }
    }

    private fun currentReferenceX(): Float = when (orientationMode) {
        OrientationMode.FOLLOW_ROBOT -> latestPose.x.toFloat()
        OrientationMode.MAGNETIC_NORTH_OVERVIEW -> overviewCenterX
    }

    private fun currentReferenceY(): Float = when (orientationMode) {
        OrientationMode.FOLLOW_ROBOT -> latestPose.y.toFloat()
        OrientationMode.MAGNETIC_NORTH_OVERVIEW -> overviewCenterY
    }

    private fun drawLiveScan(canvas: Canvas, centerX: Float, centerY: Float, scale: Float) {
        val cosYaw = cos(latestPose.theta).toFloat()
        val sinYaw = sin(latestPose.theta).toFloat()
        for (point in liveScanPoints) {
            val worldDx = cosYaw * point.x - sinYaw * point.y
            val worldDy = sinYaw * point.x + cosYaw * point.y
            val worldX = latestPose.x.toFloat() + worldDx
            val worldY = latestPose.y.toFloat() + worldDy
            val x = centerX + (worldX - currentReferenceX()) * scale
            val y = centerY - (worldY - currentReferenceY()) * scale
            if (x in 0f..width.toFloat() && y in 0f..height.toFloat()) {
                canvas.drawPoint(x, y, liveScanPaint)
            }
        }
    }

    private fun drawSubmaps(canvas: Canvas, centerX: Float, centerY: Float, scale: Float) {
        val fused = fusedMap ?: return
        val poseX = currentReferenceX()
        val poseY = currentReferenceY()
        val bitmapScale = fused.resolutionMetersPerPixel * scale
        if (bitmapScale <= 0f || fused.bitmap.isRecycled) return
        submapMatrixValues[Matrix.MSCALE_X] = bitmapScale
        submapMatrixValues[Matrix.MSKEW_X] = 0f
        submapMatrixValues[Matrix.MTRANS_X] =
            centerX + (fused.worldMinX - poseX) * scale
        submapMatrixValues[Matrix.MSKEW_Y] = 0f
        submapMatrixValues[Matrix.MSCALE_Y] = bitmapScale
        submapMatrixValues[Matrix.MTRANS_Y] =
            centerY - (fused.worldMaxY - poseY) * scale
        submapMatrixValues[Matrix.MPERSP_0] = 0f
        submapMatrixValues[Matrix.MPERSP_1] = 0f
        submapMatrixValues[Matrix.MPERSP_2] = 1f
        submapMatrix.setValues(submapMatrixValues)
        if (pointCloudVisible) {
            canvas.drawBitmap(fused.bitmap, submapMatrix, submapPaint)
        }
        val heatMap = heatMapOverlayBitmap
        if (heatMapOverlayVisible && heatMap != null && !heatMap.isRecycled &&
            heatMap.width == fused.bitmap.width && heatMap.height == fused.bitmap.height) {
            canvas.drawBitmap(heatMap, submapMatrix, heatMapPaint)
        }
        val floorPlan = floorPlanOverlayBitmap
        if (floorPlanOverlayVisible && floorPlan != null && !floorPlan.isRecycled &&
            floorPlan.width == fused.bitmap.width && floorPlan.height == fused.bitmap.height) {
            canvas.drawBitmap(floorPlan, submapMatrix, floorPlanOverlayPaint)
        }
    }

    private fun drawLiveSubmapOverlays(
        canvas: Canvas,
        centerX: Float,
        centerY: Float,
        scale: Float
    ) {
        val poseX = currentReferenceX()
        val poseY = currentReferenceY()
        for (overlay in liveSubmapOverlays.values) {
            val texture = overlay.texture
            if (overlay.bitmap.isRecycled) continue
            val bitmapScale = texture.resolution * scale
            val cosTheta = cos(texture.theta).toFloat()
            val sinTheta = sin(texture.theta).toFloat()
            submapMatrixValues[Matrix.MSCALE_X] = sinTheta * bitmapScale
            submapMatrixValues[Matrix.MSKEW_X] = -cosTheta * bitmapScale
            submapMatrixValues[Matrix.MTRANS_X] =
                centerX + (texture.originX - poseX) * scale
            submapMatrixValues[Matrix.MSKEW_Y] = cosTheta * bitmapScale
            submapMatrixValues[Matrix.MSCALE_Y] = sinTheta * bitmapScale
            submapMatrixValues[Matrix.MTRANS_Y] =
                centerY - (texture.originY - poseY) * scale
            submapMatrixValues[Matrix.MPERSP_0] = 0f
            submapMatrixValues[Matrix.MPERSP_1] = 0f
            submapMatrixValues[Matrix.MPERSP_2] = 1f
            submapMatrix.setValues(submapMatrixValues)
            canvas.drawBitmap(overlay.bitmap, submapMatrix, submapPaint)
        }
    }

    private fun requestFusedMapRender() {
        fusionGeneration++
        fusionPending = true
        startNextFusedMapRender()
    }

    private fun startNextFusedMapRender() {
        if (fusionRunning || !fusionPending || submaps.isEmpty()) return
        val nowMs = SystemClock.elapsedRealtime()
        val remainingDelayMs = LIVE_FUSION_MIN_INTERVAL_MS -
            (nowMs - lastFusionStartMs)
        if (remainingDelayMs > 0L) {
            if (!fusionThrottleScheduled) {
                fusionThrottleScheduled = true
                postDelayed(fusionThrottleRunnable, remainingDelayMs)
            }
            return
        }
        removeCallbacks(fusionThrottleRunnable)
        fusionThrottleScheduled = false
        fusionPending = false
        fusionRunning = true
        lastFusionStartMs = nowMs
        val generation = fusionGeneration
        val snapshot = submaps.values.toList()
        fusionExecutor.execute {
            val rendered = try {
                FusedMapRenderer.render(snapshot)
            } catch (_: Throwable) {
                null
            }
            post {
                fusionRunning = false
                if (generation == fusionGeneration && rendered != null) {
                    val previous = fusedMap
                    fusedMap = rendered
                    if (previous?.bitmap !== rendered.bitmap) {
                        previous?.bitmap?.recycle()
                    }
                    // Keep the two active textures as a stable foreground.
                    // Removing them here made the view alternate between the
                    // direct and fused render styles on every update, which
                    // was perceived as continuous flashing.
                    invalidate()
                } else {
                    rendered?.bitmap?.recycle()
                }
                startNextFusedMapRender()
            }
        }
    }

    private fun recycleLiveSubmapOverlays() {
        liveSubmapOverlays.values.forEach { it.bitmap.recycle() }
        liveSubmapOverlays.clear()
    }

    private data class LiveSubmapOverlay(
        val texture: SubmapTexture,
        val bitmap: Bitmap
    )

    private fun drawMapMeasurement(canvas: Canvas, centerX: Float, centerY: Float, scale: Float) {
        val measurement = mapMeasurement ?: return
        if (measurement.xSizeMeters <= 0f || measurement.ySizeMeters <= 0f) return

        if (measurement.hasSharedFloorPlanGeometry()) {
            drawSharedFloorPlanMeasurement(canvas, centerX, centerY, scale, measurement)
            return
        }

        val poseX = currentReferenceX()
        val poseY = currentReferenceY()
        val left = centerX + (measurement.minX - poseX) * scale
        val right = centerX + (measurement.maxX - poseX) * scale
        val top = centerY - (measurement.maxY - poseY) * scale
        val bottom = centerY - (measurement.minY - poseY) * scale
        val offsetPx = 30f * resources.displayMetrics.density
        val extensionPx = 7f * resources.displayMetrics.density

        canvas.drawRect(left, top, right, bottom, measurementGuidePaint)

        val xDimensionY = bottom + offsetPx
        canvas.drawLine(left, bottom, left, xDimensionY + extensionPx, measurementGuidePaint)
        canvas.drawLine(right, bottom, right, xDimensionY + extensionPx, measurementGuidePaint)
        val xName = if (measurement.xSizeMeters >= measurement.ySizeMeters) "长" else "宽"
        drawDoubleArrow(
            canvas,
            left,
            xDimensionY,
            right,
            xDimensionY,
            "$xName：${formatMeters(measurement.xSizeMeters)} m"
        )

        val yDimensionX = right + offsetPx
        canvas.drawLine(right, top, yDimensionX + extensionPx, top, measurementGuidePaint)
        canvas.drawLine(right, bottom, yDimensionX + extensionPx, bottom, measurementGuidePaint)
        val yName = if (measurement.ySizeMeters > measurement.xSizeMeters) "长" else "宽"
        drawDoubleArrow(
            canvas,
            yDimensionX,
            bottom,
            yDimensionX,
            top,
            "$yName：${formatMeters(measurement.ySizeMeters)} m"
        )
    }

    private fun drawSharedFloorPlanMeasurement(
        canvas: Canvas,
        centerX: Float,
        centerY: Float,
        scale: Float,
        measurement: MapMeasurement
    ) {
        val poseX = currentReferenceX()
        val poseY = currentReferenceY()
        fun screen(worldX: Float, worldY: Float): PointF = PointF(
            centerX + (worldX - poseX) * scale,
            centerY - (worldY - poseY) * scale
        )

        // The yellow outline on the live/global map is the exact same closed
        // polygon that C++ draws in green in the exported floor plan.
        val outlinePath = Path()
        measurement.outlineWorld.forEachIndexed { index, point ->
            val screenPoint = screen(point.x, point.y)
            if (index == 0) outlinePath.moveTo(screenPoint.x, screenPoint.y)
            else outlinePath.lineTo(screenPoint.x, screenPoint.y)
        }
        outlinePath.close()
        canvas.drawPath(outlinePath, measurementGuidePaint)

        val halfLong = measurement.orientedLongSizeMeters * 0.5f
        val halfShort = measurement.orientedShortSizeMeters * 0.5f
        val dimensionOffsetMeters =
            (30f * resources.displayMetrics.density) / scale.coerceAtLeast(1f)
        val extensionMeters =
            (7f * resources.displayMetrics.density) / scale.coerceAtLeast(1f)

        fun point(longOffset: Float, shortOffset: Float): PointF = screen(
            measurement.orientedCenterX + measurement.longAxisX * longOffset +
                measurement.shortAxisX * shortOffset,
            measurement.orientedCenterY + measurement.longAxisY * longOffset +
                measurement.shortAxisY * shortOffset
        )

        val longLabel = "长：${formatMeters(measurement.lengthMeters)} m"
        val shortLabel = "宽：${formatMeters(measurement.widthMeters)} m"
        for (side in floatArrayOf(-1f, 1f)) {
            val edgeShort = side * halfShort
            val arrowShort = side * (halfShort + dimensionOffsetMeters)
            val guideShort = side * (halfShort + dimensionOffsetMeters + extensionMeters)
            val edgeStart = point(-halfLong, edgeShort)
            val edgeEnd = point(halfLong, edgeShort)
            val guideStart = point(-halfLong, guideShort)
            val guideEnd = point(halfLong, guideShort)
            val arrowStart = point(-halfLong, arrowShort)
            val arrowEnd = point(halfLong, arrowShort)
            canvas.drawLine(edgeStart.x, edgeStart.y, guideStart.x, guideStart.y,
                measurementGuidePaint)
            canvas.drawLine(edgeEnd.x, edgeEnd.y, guideEnd.x, guideEnd.y,
                measurementGuidePaint)
            drawDoubleArrow(canvas, arrowStart.x, arrowStart.y, arrowEnd.x, arrowEnd.y,
                longLabel)
        }
        for (side in floatArrayOf(-1f, 1f)) {
            val edgeLong = side * halfLong
            val arrowLong = side * (halfLong + dimensionOffsetMeters)
            val guideLong = side * (halfLong + dimensionOffsetMeters + extensionMeters)
            val edgeStart = point(edgeLong, -halfShort)
            val edgeEnd = point(edgeLong, halfShort)
            val guideStart = point(guideLong, -halfShort)
            val guideEnd = point(guideLong, halfShort)
            val arrowStart = point(arrowLong, -halfShort)
            val arrowEnd = point(arrowLong, halfShort)
            canvas.drawLine(edgeStart.x, edgeStart.y, guideStart.x, guideStart.y,
                measurementGuidePaint)
            canvas.drawLine(edgeEnd.x, edgeEnd.y, guideEnd.x, guideEnd.y,
                measurementGuidePaint)
            drawDoubleArrow(canvas, arrowStart.x, arrowStart.y, arrowEnd.x, arrowEnd.y,
                shortLabel)
        }
    }

    private fun drawDoubleArrow(
        canvas: Canvas,
        startX: Float,
        startY: Float,
        endX: Float,
        endY: Float,
        label: String
    ) {
        canvas.drawLine(startX, startY, endX, endY, measurementPaint)
        val length = hypot(endX - startX, endY - startY)
        if (length <= 1f) return
        val ux = (endX - startX) / length
        val uy = (endY - startY) / length
        val px = -uy
        val py = ux
        val arrowLength = 11f * resources.displayMetrics.density
        val arrowHalfWidth = 5f * resources.displayMetrics.density

        canvas.drawLine(
            startX,
            startY,
            startX + ux * arrowLength + px * arrowHalfWidth,
            startY + uy * arrowLength + py * arrowHalfWidth,
            measurementPaint
        )
        canvas.drawLine(
            startX,
            startY,
            startX + ux * arrowLength - px * arrowHalfWidth,
            startY + uy * arrowLength - py * arrowHalfWidth,
            measurementPaint
        )
        canvas.drawLine(
            endX,
            endY,
            endX - ux * arrowLength + px * arrowHalfWidth,
            endY - uy * arrowLength + py * arrowHalfWidth,
            measurementPaint
        )
        canvas.drawLine(
            endX,
            endY,
            endX - ux * arrowLength - px * arrowHalfWidth,
            endY - uy * arrowLength - py * arrowHalfWidth,
            measurementPaint
        )

        val midX = (startX + endX) * 0.5f
        val midY = (startY + endY) * 0.5f
        var labelAngle = Math.toDegrees(
            atan2(endY - startY, endX - startX).toDouble()
        ).toFloat()
        while (labelAngle > 90f) labelAngle -= 180f
        while (labelAngle <= -90f) labelAngle += 180f
        canvas.save()
        canvas.rotate(labelAngle, midX, midY)
        val textWidth = measurementTextPaint.measureText(label)
        val metrics = measurementTextPaint.fontMetrics
        val paddingX = 6f * resources.displayMetrics.density
        val paddingY = 3f * resources.displayMetrics.density
        val background = RectF(
            midX - textWidth * 0.5f - paddingX,
            midY + metrics.ascent - paddingY,
            midX + textWidth * 0.5f + paddingX,
            midY + metrics.descent + paddingY
        )
        canvas.drawRoundRect(background, 5f, 5f, measurementTextBackgroundPaint)
        canvas.drawText(label, midX, midY, measurementTextPaint)
        canvas.restore()
    }

    private fun formatMeters(value: Float): String = String.format(Locale.US, "%.2f", value)


    private fun drawGrid(canvas: Canvas, centerX: Float, centerY: Float, scale: Float) {
        var x = centerX % scale
        while (x < width) {
            canvas.drawLine(x, 0f, x, height.toFloat(), gridPaint)
            x += scale
        }

        var y = centerY % scale
        while (y < height) {
            canvas.drawLine(0f, y, width.toFloat(), y, gridPaint)
            y += scale
        }
    }

    private fun squaredDistance(a: PointF, b: PointF): Float {
        val dx = a.x - b.x
        val dy = a.y - b.y
        return dx * dx + dy * dy
    }

    private fun SubmapTexture.samePoseAs(other: SubmapTexture): Boolean {
        return abs(originX - other.originX) < 1e-4f &&
            abs(originY - other.originY) < 1e-4f &&
            abs(theta - other.theta) < 1e-4f
    }

    companion object {
        private const val FINALIZED_UNKNOWN_COLOR = 0xFF9A9A9A.toInt()
        private const val LIVE_FUSION_MIN_INTERVAL_MS = 1_200L
        private const val BASE_SCALE = 65f
        private const val MIN_USER_ZOOM = 0.08f
        private const val MAX_USER_ZOOM = 6f
        private const val MAP_MIRROR_X = 1f
    }
}
