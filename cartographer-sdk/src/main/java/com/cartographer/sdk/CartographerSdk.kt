package com.cartographer.sdk

import android.app.Activity
import android.app.PendingIntent
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.graphics.Bitmap
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.hardware.usb.UsbDevice
import android.hardware.usb.UsbManager
import android.os.Build
import android.os.Handler
import android.os.Looper
import com.cartographer.demo.CartographerNative
import com.cartographer.demo.FloorPlanImageAnnotator
import com.cartographer.demo.FloorPlanMapExporter
import com.cartographer.demo.FloorPlanNative
import com.cartographer.demo.FloorPlanPixelPoint
import com.cartographer.demo.FusedMapRenderer
import com.cartographer.demo.LidarParser
import com.hoho.android.usbserial.driver.UsbSerialPort
import com.hoho.android.usbserial.driver.UsbSerialProber
import com.hoho.android.usbserial.util.SerialInputOutputManager
import java.io.File
import java.io.FileOutputStream
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicBoolean

/** Entry point for the Cartographer device SDK. One instance controls one lidar. */
class CartographerSdk private constructor(
    context: Context,
    private val config: SdkConfig,
    private val listener: CartographerListener
) : AutoCloseable, SensorEventListener {

    private val appContext = context.applicationContext
    private val mainHandler = Handler(Looper.getMainLooper())
    private val worker = Executors.newSingleThreadExecutor { runnable ->
        Thread(runnable, "cartographer-sdk").apply { isDaemon = true }
    }
    private val sensorManager = appContext.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    private val usbManager = appContext.getSystemService(Context.USB_SERVICE) as UsbManager
    private val native = CartographerNative()
    private val floorPlan = FloorPlanNative()
    private val parser = LidarParser(native)
    private val closed = AtomicBoolean(false)
    private val mapping = AtomicBoolean(false)
    private val sensorsRunning = AtomicBoolean(false)

    @Volatile private var serialPort: UsbSerialPort? = null
    @Volatile private var ioManager: SerialInputOutputManager? = null
    @Volatile private var initialized = false
    private var receiverRegistered = false
    private val acceleration = FloatArray(3)
    private val gyroscope = FloatArray(3)
    private var hasAcceleration = false
    private var hasGyroscope = false
    private var lastImuTimestampNs = 0L
    private var lastPublishedScanTimestampNs = 0L

    private val eventReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context?, intent: Intent?) {
            when (intent?.action) {
                ACTION_USB_PERMISSION -> {
                    val granted = intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)
                    post { listener.onUsbPermissionResult(granted) }
                    if (!granted) report(SdkError.PERMISSION_DENIED, "USB permission was denied")
                }
                UsbManager.ACTION_USB_DEVICE_ATTACHED -> {
                    val device = intent.usbDevice() ?: return
                    if (isSupported(device)) post { listener.onUsbDeviceFound(device.deviceName) }
                }
                UsbManager.ACTION_USB_DEVICE_DETACHED -> {
                    val device = intent.usbDevice() ?: return
                    if (serialPort?.device?.deviceId == device.deviceId) {
                        worker.execute { disconnectInternal(notify = true) }
                        report(SdkError.DEVICE_DETACHED, "Lidar was detached")
                    }
                }
            }
        }
    }

    /** Finds the supported CH340 lidar and asks Android for permission when needed. */
    fun requestUsbPermission(activity: Activity): SdkResult {
        if (closed.get()) return SdkResult.failure(SdkError.CLOSED, "SDK is closed")
        val device = findSupportedDevice()
            ?: return SdkResult.failure(SdkError.DEVICE_NOT_FOUND, "CH340 lidar not found")
        post { listener.onUsbDeviceFound(device.deviceName) }
        if (usbManager.hasPermission(device)) {
            post { listener.onUsbPermissionResult(true) }
            return SdkResult.success()
        }
        val flags = PendingIntent.FLAG_UPDATE_CURRENT or
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) PendingIntent.FLAG_MUTABLE else 0
        val pending = PendingIntent.getBroadcast(
            activity,
            config.permissionRequestCode,
            Intent(ACTION_USB_PERMISSION).setPackage(appContext.packageName),
            flags
        )
        usbManager.requestPermission(device, pending)
        return SdkResult.success()
    }

    /** Opens the current supported lidar. Permission must already be granted. */
    fun connect(): SdkResult {
        if (closed.get()) return SdkResult.failure(SdkError.CLOSED, "SDK is closed")
        val device = findSupportedDevice()
            ?: return SdkResult.failure(SdkError.DEVICE_NOT_FOUND, "CH340 lidar not found")
        if (!usbManager.hasPermission(device)) {
            return SdkResult.failure(SdkError.PERMISSION_REQUIRED, "Call requestUsbPermission first")
        }
        postState(ConnectionState.CONNECTING)
        worker.execute { openDevice(device) }
        return SdkResult.success()
    }

    fun disconnect(): SdkResult {
        if (closed.get()) return SdkResult.failure(SdkError.CLOSED, "SDK is closed")
        worker.execute { disconnectInternal(notify = true) }
        return SdkResult.success()
    }

    /** Starts phone IMU collection and a new SLAM trajectory. */
    fun startMapping(): SdkResult {
        if (closed.get()) return SdkResult.failure(SdkError.CLOSED, "SDK is closed")
        if (!initialized) return SdkResult.failure(SdkError.NOT_INITIALIZED, "SDK initialization failed")
        if (!mapping.compareAndSet(false, true)) return SdkResult.failure(SdkError.ALREADY_MAPPING, "Mapping already started")
        val accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        val gyroscopeSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        if (accelerometer == null || gyroscopeSensor == null) {
            mapping.set(false)
            return SdkResult.failure(SdkError.SENSOR_UNAVAILABLE, "Accelerometer or gyroscope unavailable")
        }
        native.startTrajectory()
        hasAcceleration = false
        hasGyroscope = false
        lastImuTimestampNs = 0L
        sensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_FASTEST)
        sensorManager.registerListener(this, gyroscopeSensor, SensorManager.SENSOR_DELAY_FASTEST)
        sensorsRunning.set(true)
        parser.enableLidarAfterImuReady()
        post { listener.onMappingStateChanged(MappingState.MAPPING) }
        scheduleSnapshot()
        return SdkResult.success()
    }

    fun stopMapping(): SdkResult {
        if (closed.get()) return SdkResult.failure(SdkError.CLOSED, "SDK is closed")
        if (!mapping.compareAndSet(true, false)) return SdkResult.failure(SdkError.NOT_MAPPING, "Mapping is not running")
        stopSensors()
        worker.execute {
            native.finishTrajectory()
            post { listener.onMappingStateChanged(MappingState.FINISHED) }
        }
        return SdkResult.success()
    }

    fun saveMap(output: File, includeUnfinishedSubmaps: Boolean = false, callback: (SdkResult) -> Unit) {
        executeFileOperation(callback) {
            output.parentFile?.mkdirs()
            if (native.saveMap(output.absolutePath, includeUnfinishedSubmaps)) SdkResult.success()
            else SdkResult.failure(SdkError.IO_ERROR, "Map could not be saved")
        }
    }

    fun loadMap(input: File, callback: (SdkResult) -> Unit) {
        executeFileOperation(callback) {
            when {
                mapping.get() -> SdkResult.failure(SdkError.ALREADY_MAPPING, "Stop mapping before loading")
                !input.isFile -> SdkResult.failure(SdkError.IO_ERROR, "Map file does not exist")
                native.loadMap(input.absolutePath, true) -> SdkResult.success()
                else -> SdkResult.failure(SdkError.IO_ERROR, "Map could not be loaded")
            }
        }
    }

    fun getSubmapTextures(callback: (List<SubmapTexture>) -> Unit): SdkResult {
        if (closed.get()) return SdkResult.failure(SdkError.CLOSED, "SDK is closed")
        worker.execute {
            val textures = native.getSubmapTextures().map(::toSdkSubmapTexture)
            post { callback(textures) }
        }
        return SdkResult.success()
    }

    /** Returns only the newest Cartographer submap texture. Callback runs on the main thread. */
    fun getLatestSubmapTexture(callback: (SubmapTexture?) -> Unit): SdkResult {
        if (closed.get()) return SdkResult.failure(SdkError.CLOSED, "SDK is closed")
        worker.execute {
            val texture = native.getLatestSubmapTexture()?.let(::toSdkSubmapTexture)
            post { callback(texture) }
        }
        return SdkResult.success()
    }

    /** Returns the overlapping active submaps used for efficient live-map refreshes. */
    fun getActiveSubmapTextures(callback: (List<SubmapTexture>) -> Unit): SdkResult {
        if (closed.get()) return SdkResult.failure(SdkError.CLOSED, "SDK is closed")
        worker.execute {
            val textures = native.getActiveSubmapTextures().map(::toSdkSubmapTexture)
            post { callback(textures) }
        }
        return SdkResult.success()
    }

    fun getTrajectory(callback: (List<PosePoint>) -> Unit): SdkResult {
        if (closed.get()) return SdkResult.failure(SdkError.CLOSED, "SDK is closed")
        worker.execute {
            val points = native.getTrajectoryNodePoses().map { PosePoint(it.x, it.y) }
            post { callback(points) }
        }
        return SdkResult.success()
    }

    /**
     * Fuses every optimized submap into the neutral gray/white/black raster used
     * by the final map display. The caller owns [FinalRasterMap.bitmap] and must
     * recycle it when it is no longer needed.
     */
    fun getFinalRasterMap(
        output: File? = null,
        callback: (SdkResult, FinalRasterMap?) -> Unit
    ): SdkResult {
        if (closed.get()) {
            val failure = SdkResult.failure(SdkError.CLOSED, "SDK is closed")
            post { callback(failure, null) }
            return failure
        }
        worker.execute {
            val outcome = runCatching {
                val fused = renderFinalMap()
                    ?: throw IllegalStateException("No optimized submaps are available")
                try {
                    val bitmap = fused.bitmap.copy(Bitmap.Config.ARGB_8888, false)
                        ?: throw IllegalStateException("Final raster bitmap copy failed")
                    if (output != null && !writePng(output, bitmap)) {
                        bitmap.recycle()
                        throw IllegalStateException("Final raster PNG could not be written")
                    }
                    FinalRasterMap(
                        bitmap = bitmap,
                        outputFile = output,
                        geometry = fused.toSdkGeometry()
                    )
                } finally {
                    fused.bitmap.recycle()
                }
            }
            val value = outcome.getOrNull()
            val result = if (value != null) SdkResult.success() else {
                SdkResult.failure(
                    SdkError.FINAL_MAP_FAILED,
                    outcome.exceptionOrNull()?.message ?: "Final raster generation failed"
                )
            }
            post { callback(result, value) }
        }
        return SdkResult.success()
    }

    /**
     * Generates the latest fitted floor-plan PNG and writes the four-side size
     * annotation into the same file. The returned dimensions and world outline
     * come from the exact green polygon drawn in that PNG.
     */
    fun generateFloorPlan(
        output: File,
        workDir: File,
        callback: (SdkResult, FinalFloorPlan?) -> Unit
    ): SdkResult {
        if (closed.get()) {
            val failure = SdkResult.failure(SdkError.CLOSED, "SDK is closed")
            post { callback(failure, null) }
            return failure
        }
        worker.execute {
            val outcome = runCatching { generateFinalFloorPlan(output, workDir) }
            val value = outcome.getOrNull()
            val result = if (value != null) SdkResult.success() else {
                SdkResult.failure(
                    SdkError.FLOOR_PLAN_FAILED,
                    outcome.exceptionOrNull()?.message ?: "Floor plan generation failed"
                )
            }
            post { callback(result, value) }
        }
        return SdkResult.success()
    }

    fun getRelocalizationStatus(callback: (RelocalizationStatus) -> Unit): SdkResult {
        if (closed.get()) return SdkResult.failure(SdkError.CLOSED, "SDK is closed")
        worker.execute {
            val status = native.getRelocalizationStatus()
            post {
                callback(
                    RelocalizationStatus(
                        status.oldTrajectoryCount, status.activeTrajectoryId,
                        status.activeNodeCount, status.interTrajectoryConstraintCount,
                        status.matchedActiveNodeCount, status.matchedOldSubmapCount,
                        status.isRelocalized
                    )
                )
            }
        }
        return SdkResult.success()
    }

    override fun onSensorChanged(event: SensorEvent) {
        if (!mapping.get()) return
        when (event.sensor.type) {
            Sensor.TYPE_ACCELEROMETER -> {
                event.values.copyInto(acceleration, endIndex = 3)
                hasAcceleration = true
            }
            Sensor.TYPE_GYROSCOPE -> {
                event.values.copyInto(gyroscope, endIndex = 3)
                hasGyroscope = true
            }
            else -> return
        }
        if (event.sensor.type == Sensor.TYPE_GYROSCOPE && hasAcceleration && hasGyroscope &&
            event.timestamp - lastImuTimestampNs >= config.imuMinimumIntervalNs) {
            lastImuTimestampNs = event.timestamp
            native.addImuData(event.timestamp, acceleration, gyroscope)
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit

    override fun close() {
        if (!closed.compareAndSet(false, true)) return
        mapping.set(false)
        stopSensors()
        disconnectInternal(notify = false)
        if (receiverRegistered) {
            runCatching { appContext.unregisterReceiver(eventReceiver) }
            receiverRegistered = false
        }
        native.reset()
        worker.shutdownNow()
        postState(ConnectionState.CLOSED)
    }

    private fun initializeInternal() {
        registerReceiver()
        postState(ConnectionState.DISCONNECTED)
        worker.execute {
            val configDir = runCatching { unpackConfiguration() }.getOrElse {
                report(SdkError.CONFIGURATION_ERROR, "Configuration extraction failed", it)
                return@execute
            }
            initialized = native.initialize(configDir.absolutePath, "android_2d.lua")
            if (!initialized) report(SdkError.INITIALIZATION_FAILED, "Native SLAM initialization failed")
            else post { listener.onMappingStateChanged(MappingState.READY) }
        }
    }

    private fun registerReceiver() {
        val filter = IntentFilter().apply {
            addAction(ACTION_USB_PERMISSION)
            addAction(UsbManager.ACTION_USB_DEVICE_ATTACHED)
            addAction(UsbManager.ACTION_USB_DEVICE_DETACHED)
        }
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            appContext.registerReceiver(eventReceiver, filter, Context.RECEIVER_NOT_EXPORTED)
        } else {
            @Suppress("DEPRECATION") appContext.registerReceiver(eventReceiver, filter)
        }
        receiverRegistered = true
    }

    private fun openDevice(device: UsbDevice) {
        if (closed.get()) return
        disconnectInternal(notify = false)
        try {
            val driver = UsbSerialProber.getDefaultProber().probeDevice(device)
                ?: throw IllegalStateException("No CH340 serial driver")
            val connection = usbManager.openDevice(device)
                ?: throw SecurityException("USB permission is missing")
            val port = driver.ports.firstOrNull() ?: throw IllegalStateException("No serial port")
            port.open(connection)
            port.setParameters(BAUD_RATE, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE)
            port.dtr = false
            serialPort = port
            Thread.sleep(200)
            sendCommand(port, 0xF0, 0x0F)
            ioManager = SerialInputOutputManager(port, object : SerialInputOutputManager.Listener {
                override fun onNewData(data: ByteArray) {
                    if (closed.get()) return
                    runCatching { parser.onDataReceived(data, data.size) }
                        .onFailure { report(SdkError.PROTOCOL_ERROR, "Lidar frame parsing failed", it) }
                    parser.getLatestScan()?.takeIf { it.timestampNs != lastPublishedScanTimestampNs }?.let { scan ->
                        lastPublishedScanTimestampNs = scan.timestampNs
                        post {
                            listener.onLidarScan(
                                LidarScan(scan.timestampNs, scan.ranges.copyOf(), scan.anglesDeg.copyOf())
                            )
                        }
                    }
                }
                override fun onRunError(error: Exception) {
                    report(SdkError.SERIAL_ERROR, "Serial reader stopped", error)
                    postState(ConnectionState.ERROR)
                }
            }).also { it.start() }
            postState(ConnectionState.CONNECTED)
        } catch (error: Exception) {
            disconnectInternal(notify = false)
            postState(ConnectionState.ERROR)
            report(SdkError.SERIAL_ERROR, "Could not open CH340 lidar", error)
        }
    }

    private fun disconnectInternal(notify: Boolean) {
        ioManager?.stop()
        ioManager = null
        serialPort?.let { port ->
            runCatching { sendCommand(port, 0xF5, 0x0A) }
            runCatching { port.close() }
        }
        serialPort = null
        parser.clearLatestScanAndPendingData()
        if (notify) postState(ConnectionState.DISCONNECTED)
    }

    private fun scheduleSnapshot() {
        mainHandler.postDelayed(object : Runnable {
            override fun run() {
                if (!mapping.get() || closed.get()) return
                val pose = native.getCurrentPose()?.to2D()
                val status = native.getStatus()
                listener.onSnapshot(
                    SdkSnapshot(
                        pose = pose?.let { Pose2D(it.x, it.y, it.theta) },
                        rangeFrames = status.rangeFrames,
                        imuSamples = status.imuSamples,
                        insertedNodes = status.insertedNodes,
                        hasPose = status.hasPose
                    )
                )
                mainHandler.postDelayed(this, config.snapshotIntervalMs)
            }
        }, config.snapshotIntervalMs)
    }

    private fun unpackConfiguration(): File {
        val output = File(appContext.noBackupFilesDir, "cartographer-sdk/config-$SDK_VERSION")
        if (File(output, ".complete").isFile) return output
        output.deleteRecursively()
        output.mkdirs()
        val key = byteArrayOf(0x43, 0x61, 0x72, 0x74, 0x6f, 0x53, 0x44, 0x4b)
        appContext.assets.list("carto_cfg").orEmpty().filter { it.endsWith(".bin") }.forEach { assetName ->
            val encrypted = appContext.assets.open("carto_cfg/$assetName").use { it.readBytes() }
            for (index in encrypted.indices) encrypted[index] =
                (encrypted[index].toInt() xor key[index % key.size].toInt()).toByte()
            val plainName = assetName.removeSuffix(".bin")
            File(output, plainName).writeBytes(encrypted)
        }
        require(File(output, "android_2d.lua").isFile) { "android_2d.lua is missing" }
        File(output, ".complete").writeText(SDK_VERSION)
        return output
    }

    private fun stopSensors() {
        if (sensorsRunning.compareAndSet(true, false)) sensorManager.unregisterListener(this)
    }

    private fun findSupportedDevice(): UsbDevice? =
        usbManager.deviceList.values.firstOrNull(::isSupported)

    private fun isSupported(device: UsbDevice) =
        device.vendorId == SUPPORTED_VENDOR_ID && device.productId == SUPPORTED_PRODUCT_ID

    private fun sendCommand(port: UsbSerialPort, first: Int, second: Int) {
        port.write(byteArrayOf(0xAA.toByte(), 0x55.toByte(), first.toByte(), second.toByte()), 100)
    }

    private fun toSdkSubmapTexture(
        texture: com.cartographer.demo.SubmapTexture
    ) = SubmapTexture(
        width = texture.width,
        height = texture.height,
        version = texture.version,
        trajectoryId = texture.trajectoryId,
        submapIndex = texture.submapIndex,
        resolution = texture.resolution,
        originX = texture.originX,
        originY = texture.originY,
        theta = texture.theta,
        pixels = texture.pixels.copyOf(),
        rawCells = texture.rawCells.copyOf()
    )

    private fun renderFinalMap(): FusedMapRenderer.Result? =
        FusedMapRenderer.render(native.getSubmapTextures(), finalized = true)

    private fun FusedMapRenderer.Result.toSdkGeometry() = RasterGeometry(
        resolutionMetersPerPixel = resolutionMetersPerPixel,
        worldMinX = worldMinX,
        worldMaxX = worldMinX + bitmap.width * resolutionMetersPerPixel,
        worldMinY = worldMaxY - bitmap.height * resolutionMetersPerPixel,
        worldMaxY = worldMaxY,
        contentMinX = contentMinX,
        contentMaxX = contentMaxX,
        contentMinY = contentMinY,
        contentMaxY = contentMaxY,
        widthPixels = bitmap.width,
        heightPixels = bitmap.height
    )

    private fun generateFinalFloorPlan(output: File, workDir: File): FinalFloorPlan {
        val fused = renderFinalMap()
            ?: throw IllegalStateException("No optimized submaps are available")
        workDir.mkdirs()
        output.parentFile?.mkdirs()
        var inputResource: FloorPlanMapExporter.RenderResult? = null
        var visualResource: FloorPlanMapExporter.RenderResult? = null
        var semanticResource: FloorPlanMapExporter.RenderResult? = null
        try {
            inputResource = FloorPlanMapExporter.renderFusedWithGeometry(
                fused,
                FloorPlanMapExporter.Style.ALGORITHM_INPUT
            ) ?: throw IllegalStateException("Algorithm-input raster generation failed")
            visualResource = FloorPlanMapExporter.renderFusedWithGeometry(
                fused,
                FloorPlanMapExporter.Style.VISUAL_MAP
            ) ?: throw IllegalStateException("Visual raster generation failed")
            semanticResource = FloorPlanMapExporter.renderFusedWithGeometry(
                fused,
                FloorPlanMapExporter.Style.SEMANTIC
            ) ?: throw IllegalStateException("Semantic raster generation failed")
            val inputRender = requireNotNull(inputResource)
            val visualRender = requireNotNull(visualResource)
            val semanticRender = requireNotNull(semanticResource)
            val inputFile = File(workDir, "floorplan_input.png")
            val visualFile = File(workDir, "floorplan_visual.png")
            val semanticFile = File(workDir, "floorplan_semantic.png")
            if (!writePng(inputFile, inputRender.bitmap) ||
                !writePng(visualFile, visualRender.bitmap) ||
                !writePng(semanticFile, semanticRender.bitmap)) {
                throw IllegalStateException("Floor-plan input rasters could not be written")
            }

            val geometry = inputRender.geometry
            val trajectoryPixels = native.getTrajectoryNodePoses().mapNotNull { point ->
                val pixelX = geometry.worldToPixelX(point.x)
                val pixelY = geometry.worldToPixelY(point.y)
                if (pixelX.isFinite() && pixelY.isFinite() &&
                    pixelX >= 0f && pixelY >= 0f &&
                    pixelX < geometry.widthPx && pixelY < geometry.heightPx) {
                    FloorPlanPixelPoint(pixelX, pixelY)
                } else {
                    null
                }
            }
            if (trajectoryPixels.isEmpty()) {
                throw IllegalStateException("No optimized trajectory is available")
            }
            val generation = floorPlan.generate(
                input = inputFile,
                visualInput = visualFile,
                semanticInput = semanticFile,
                output = output,
                workDir = workDir,
                metersPerPixel = geometry.resolutionMetersPerPixel,
                trajectoryPixels = trajectoryPixels
            ) ?: throw IllegalStateException("Native floor-plan fitting failed")

            val annotation = FloorPlanImageAnnotator.annotateFile(
                file = output,
                generation = generation,
                metersPerPixel = geometry.resolutionMetersPerPixel
            )
            val dimensions = FloorPlanDimensions(
                lengthMeters = annotation.lengthMeters,
                widthMeters = annotation.widthMeters,
                areaSquareMeters = generation.footprintAreaPixelsSquared *
                    geometry.resolutionMetersPerPixel *
                    geometry.resolutionMetersPerPixel
            )
            if (!dimensions.isValid()) {
                throw IllegalStateException(
                    annotation.failureReason ?: "Floor-plan dimensions are invalid"
                )
            }
            val outlineWorld = generation.outlineVerticesPixels.map { point ->
                PosePoint(
                    x = geometry.worldMinX + point.x * geometry.resolutionMetersPerPixel,
                    y = geometry.worldMaxY - point.y * geometry.resolutionMetersPerPixel
                )
            }
            return FinalFloorPlan(
                imageFile = output,
                structuralMapFile = File(workDir, "best_structural_map.png")
                    .takeIf(File::isFile),
                dimensions = dimensions,
                dimensionsAnnotated = annotation.annotated,
                annotationFailureReason = annotation.failureReason,
                outlineClosed = generation.outlineClosed,
                outlineWorld = outlineWorld,
                rotationDegrees = generation.rotationDegrees,
                supportRatio = generation.supportRatio,
                footprintPerimeterMeters = generation.footprintPerimeterPixels *
                    geometry.resolutionMetersPerPixel,
                geometry = RasterGeometry(
                    resolutionMetersPerPixel = geometry.resolutionMetersPerPixel,
                    worldMinX = geometry.worldMinX,
                    worldMaxX = geometry.worldMaxX,
                    worldMinY = geometry.worldMinY,
                    worldMaxY = geometry.worldMaxY,
                    contentMinX = geometry.contentMinX,
                    contentMaxX = geometry.contentMaxX,
                    contentMinY = geometry.contentMinY,
                    contentMaxY = geometry.contentMaxY,
                    widthPixels = geometry.widthPx,
                    heightPixels = geometry.heightPx
                )
            )
        } finally {
            inputResource?.bitmap?.recycle()
            visualResource?.bitmap?.recycle()
            semanticResource?.bitmap?.recycle()
            fused.bitmap.recycle()
        }
    }

    private fun writePng(file: File, bitmap: Bitmap): Boolean {
        file.parentFile?.mkdirs()
        return FileOutputStream(file).use { stream ->
            bitmap.compress(Bitmap.CompressFormat.PNG, 100, stream)
        }
    }

    private fun executeFileOperation(callback: (SdkResult) -> Unit, operation: () -> SdkResult) {
        if (closed.get()) {
            post { callback(SdkResult.failure(SdkError.CLOSED, "SDK is closed")) }
            return
        }
        worker.execute {
            val result = runCatching(operation).getOrElse {
                SdkResult.failure(SdkError.IO_ERROR, it.message ?: "File operation failed")
            }
            post { callback(result) }
        }
    }

    private fun report(code: SdkError, message: String, cause: Throwable? = null) =
        post { listener.onError(code, message, cause) }

    private fun postState(state: ConnectionState) = post { listener.onConnectionStateChanged(state) }
    private fun post(action: () -> Unit) {
        if (Looper.myLooper() == Looper.getMainLooper()) action() else mainHandler.post(action)
    }

    @Suppress("DEPRECATION")
    private fun Intent.usbDevice(): UsbDevice? =
        if (Build.VERSION.SDK_INT >= 33) getParcelableExtra(UsbManager.EXTRA_DEVICE, UsbDevice::class.java)
        else getParcelableExtra(UsbManager.EXTRA_DEVICE)

    companion object {
        const val SDK_VERSION = "1.1.0"
        const val SUPPORTED_VENDOR_ID = 0x1A86
        const val SUPPORTED_PRODUCT_ID = 0x7523
        private const val BAUD_RATE = 230400
        private const val ACTION_USB_PERMISSION = "com.cartographer.sdk.USB_PERMISSION"

        @JvmStatic
        fun initialize(context: Context, config: SdkConfig = SdkConfig(), listener: CartographerListener): CartographerSdk =
            CartographerSdk(context, config, listener).also { it.initializeInternal() }

        /** Returns an Intent for the packaged page containing every reference-app feature. */
        @JvmStatic
        fun createFullExperienceIntent(context: Context): Intent =
            Intent(context, FullExperienceActivity::class.java)
    }
}

data class SdkConfig(
    val snapshotIntervalMs: Long = 100L,
    val imuMinimumIntervalNs: Long = 5_000_000L,
    val permissionRequestCode: Int = 4107
) {
    init {
        require(snapshotIntervalMs >= 50L) { "snapshotIntervalMs must be at least 50 ms" }
        require(imuMinimumIntervalNs > 0L) { "imuMinimumIntervalNs must be positive" }
    }
}

interface CartographerListener {
    fun onUsbDeviceFound(deviceName: String) = Unit
    fun onUsbPermissionResult(granted: Boolean) = Unit
    fun onConnectionStateChanged(state: ConnectionState) = Unit
    fun onMappingStateChanged(state: MappingState) = Unit
    fun onLidarScan(scan: LidarScan) = Unit
    fun onSnapshot(snapshot: SdkSnapshot) = Unit
    fun onError(code: SdkError, message: String, cause: Throwable?) = Unit
}

/** Java-friendly listener with no-op implementations. */
open class CartographerListenerAdapter : CartographerListener

enum class ConnectionState { DISCONNECTED, CONNECTING, CONNECTED, ERROR, CLOSED }
enum class MappingState { READY, MAPPING, FINISHED }
enum class SdkError {
    CLOSED, NOT_INITIALIZED, INITIALIZATION_FAILED, CONFIGURATION_ERROR,
    DEVICE_NOT_FOUND, PERMISSION_REQUIRED, PERMISSION_DENIED, DEVICE_DETACHED,
    SERIAL_ERROR, PROTOCOL_ERROR, SENSOR_UNAVAILABLE, ALREADY_MAPPING, NOT_MAPPING,
    IO_ERROR, FINAL_MAP_FAILED, FLOOR_PLAN_FAILED
}

data class SdkResult(val isSuccess: Boolean, val error: SdkError? = null, val message: String? = null) {
    companion object {
        @JvmStatic fun success() = SdkResult(true)
        @JvmStatic fun failure(error: SdkError, message: String) = SdkResult(false, error, message)
    }
}

data class Pose2D(val x: Double, val y: Double, val theta: Double)
data class LidarScan(val timestampNs: Long, val rangesMeters: FloatArray, val anglesDegrees: FloatArray)
data class SdkSnapshot(
    val pose: Pose2D?,
    val rangeFrames: Long,
    val imuSamples: Long,
    val insertedNodes: Long,
    val hasPose: Boolean
)

data class PosePoint(val x: Float, val y: Float)

data class SubmapTexture(
    val width: Int,
    val height: Int,
    val version: Int,
    val trajectoryId: Int,
    val submapIndex: Int,
    val resolution: Float,
    val originX: Float,
    val originY: Float,
    val theta: Float,
    val pixels: IntArray,
    /** Packed Cartographer texture cells: intensity in bits 8..15, alpha in bits 0..7. */
    val rawCells: IntArray = IntArray(0)
)

/** Pixel/world transform shared by the final raster and fitted floor plan. */
data class RasterGeometry(
    val resolutionMetersPerPixel: Float,
    val worldMinX: Float,
    val worldMaxX: Float,
    val worldMinY: Float,
    val worldMaxY: Float,
    val contentMinX: Float,
    val contentMaxX: Float,
    val contentMinY: Float,
    val contentMaxY: Float,
    val widthPixels: Int,
    val heightPixels: Int
) {
    fun worldToPixelX(worldX: Float): Float =
        (worldX - worldMinX) / resolutionMetersPerPixel

    fun worldToPixelY(worldY: Float): Float =
        (worldMaxY - worldY) / resolutionMetersPerPixel
}

data class FinalRasterMap(
    val bitmap: Bitmap,
    val outputFile: File?,
    val geometry: RasterGeometry
)

data class FloorPlanDimensions(
    val lengthMeters: Float,
    val widthMeters: Float,
    val areaSquareMeters: Float
) {
    fun isValid(): Boolean = lengthMeters.isFinite() && lengthMeters > 0f &&
        widthMeters.isFinite() && widthMeters > 0f &&
        areaSquareMeters.isFinite() && areaSquareMeters > 0f
}

data class FinalFloorPlan(
    val imageFile: File,
    val structuralMapFile: File?,
    val dimensions: FloorPlanDimensions,
    val dimensionsAnnotated: Boolean,
    val annotationFailureReason: String?,
    val outlineClosed: Boolean,
    val outlineWorld: List<PosePoint>,
    val rotationDegrees: Float,
    val supportRatio: Float,
    val footprintPerimeterMeters: Float,
    val geometry: RasterGeometry
)

data class RelocalizationStatus(
    val oldTrajectoryCount: Int,
    val activeTrajectoryId: Int,
    val activeNodeCount: Int,
    val interTrajectoryConstraintCount: Int,
    val matchedActiveNodeCount: Int,
    val matchedOldSubmapCount: Int,
    val isRelocalized: Boolean
)
