package com.cartographer.demo

import android.Manifest
import android.app.PendingIntent
import android.content.BroadcastReceiver
import android.content.ContentValues
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.pm.PackageManager
import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.hardware.usb.UsbDevice
import android.hardware.usb.UsbManager
import android.media.MediaScannerConnection
import android.net.wifi.WifiManager
import android.os.Build
import android.os.Bundle
import android.os.Environment
import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import android.provider.MediaStore
import android.util.Log
import android.view.View
import android.widget.Button
import android.widget.CheckBox
import android.widget.ImageButton
import android.widget.TextView
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import com.hoho.android.usbserial.driver.UsbSerialPort
import com.hoho.android.usbserial.driver.UsbSerialProber
import com.hoho.android.usbserial.util.SerialInputOutputManager
import java.io.File
import java.io.FileOutputStream
import java.io.IOException
import java.util.Locale

open class MainActivity : AppCompatActivity(), SensorEventListener {

    private companion object {
        // Texture generation includes probability conversion and gzip work in
        // native code.  Do not let UI refresh steal CPU/lock time from SLAM.
        private const val SUBMAP_FETCH_INTERVAL_MS = 750L
        private const val FULL_SUBMAP_FETCH_INTERVAL_MS = 30_000L
        private const val DEBUG_TEXT_UPDATE_INTERVAL_MS = 500L
        private const val RELOCALIZATION_STATUS_INTERVAL_MS = 1_000L
        private const val RELOCALIZATION_TIMEOUT_MS = 90_000L
        private const val RELOCALIZATION_TIMEOUT_NODE_COUNT = 30
        private const val LIVE_SCAN_STALE_TIMEOUT_NS = 1_000_000_000L
        private const val DEVICE_TO_ROBOT_HEADING_OFFSET_RAD = 0f
        private const val MAGNETIC_CALIBRATION_TIMEOUT_MS = 10_000L
        private const val RSSI_SAMPLE_INTERVAL_MS = 1_000L
    }

    private enum class MappingMode {
        NORMAL_MAPPING,
        CONTINUE_MAPPING
    }

    private enum class RelocalizationPhase {
        NOT_REQUIRED,
        WAITING,
        RELOCALIZED,
        TIMEOUT
    }

    private lateinit var tvPose: TextView
    private lateinit var tvDebugData: TextView // ✅ 新加：显示真实数据
    private lateinit var tvMapMeasurement: TextView
    private lateinit var mapView: LidarMapView
    private lateinit var floorPlanView: ZoomableImageView
    private lateinit var btnFinishMapping: Button
    private lateinit var btnContinueMapping: Button
    private lateinit var btnCancelContinueMapping: Button
    private lateinit var btnSaveMap: Button
    private lateinit var btnSaveFloorPlan: Button
    private lateinit var btnMapNorth: ImageButton
    private lateinit var checkPointCloud: CheckBox
    private lateinit var checkFloorPlanOverlay: CheckBox
    private lateinit var checkHeatMapOverlay: CheckBox
    private lateinit var checkTrajectoryOverlay: CheckBox
    private val carto = CartographerNative()
    private val floorPlanNative = FloorPlanNative()
    private val sensorManager by lazy { getSystemService(Context.SENSOR_SERVICE) as SensorManager }
    private val magneticHeadingProvider by lazy { MagneticHeadingProvider(sensorManager) }
    private val wifiManager by lazy {
        applicationContext.getSystemService(Context.WIFI_SERVICE) as WifiManager
    }
    private lateinit var lidarParser: LidarParser

    private val usbManager by lazy { getSystemService(Context.USB_SERVICE) as UsbManager }
    private var usbSerialPort: UsbSerialPort? = null
    private var usbIoManager: SerialInputOutputManager? = null
    private val ACTION_USB_PERMISSION = "com.cartographer.demo.USB_PERMISSION"

    private var isSensorRunning = false
    private var isMappingRunning = false
    private var isMappingFinished = false
    @Volatile private var isFinishingMapping = false
    private var mappingMode = MappingMode.NORMAL_MAPPING
    @Volatile private var relocalizationPhase = RelocalizationPhase.NOT_REQUIRED
    @Volatile private var relocalizationStatus = RelocalizationStatus()
    private var relocalizationStartedMs = 0L

    private var imuCount = 0
    private var lidarCount = 0L
    private val acc = FloatArray(3)
    private val gyro = FloatArray(3)
    private var lastImuTimeNs: Long = 0
    private var lastImuSendNs = 0L
    private var lastAccelerationSampleNs = 0L
    private var hasAccelerationSample = false
    private var hasGyroscopeSample = false
    private val IMU_MIN_INTERVAL_NS = 10 * 1000_000L
    private val IMU_MAX_PAIR_AGE_NS = 25 * 1000_000L

    // ✅ 测试用：保存最新雷达数据
    private var latestLidarPoints = 0
    private var latestLidarDist = 0f
    private var latestLidarAngle = 0f
    private var lastRenderedScanNs = 0L
    private var lastDebugTextUpdateMs = 0L
    private var lastSubmapFetchMs = 0L
    @Volatile private var lastFetchedSubmapNodeCount = -1L
    @Volatile private var lastFullSubmapFetchMs = 0L
    private var lastRelocalizationStatusFetchMs = 0L
    private var latestSubmapRefreshIntervalMs = 0L
    private var lastObservedRangeFrames = 0L
    private var lastObservedLocalSlamResults = 0L
    private var lastObservedInsertedNodes = 0L
    private var lastRangeFrameChangeMs = 0L
    private var lastLocalSlamChangeMs = 0L
    private var lastInsertedNodeChangeMs = 0L
    private var latestRangeFrameIntervalMs = 0L
    private var latestLocalSlamIntervalMs = 0L
    private var latestInsertedNodeIntervalMs = 0L
    @Volatile private var isSubmapFetchRunning = false
    @Volatile private var isRelocalizationStatusFetchRunning = false
    @Volatile private var latestSubmapCount = 0
    private var currentFloorPlanResult: File? = null
    private var currentFloorPlanDimensions: FloorPlanDimensions? = null
    private var currentFloorPlanWarning: String? = null
    private var floorPlanLayers: FloorPlanLayers? = null
    private var floorPlanDisplayBitmap: Bitmap? = null
    private var pendingLegacyFloorPlanSave: File? = null
    private var pendingLegacyMapExport: File? = null
    private val measurementTextures = LinkedHashMap<String, SubmapTexture>()
    private var continueBaseSubmaps: List<SubmapTexture> = emptyList()
    private var currentMapMeasurement: MapMeasurement? = null
    private var measurementGeneration = 0L
    private var isMeasurementCalculationRunning = false
    private var measurementCalculationDirty = false
    @Volatile private var pendingNorthAlignment: MapNorthAlignment? = null
    @Volatile private var loadedMapNorthAlignment: MapNorthAlignment? = null
    private var savedMapNorthAlignment: MapNorthAlignment? = null
    private var continueSourceMapFile: File? = null
    private var savedMapFile: File? = null
    @Volatile private var finishedMapPose: Pose2D? = null
    @Volatile private var isMagneticCalibrationRunning = false
    private val rssiSamples = ArrayList<RssiSample>()
    private var lastRssiSampleMs = 0L

    private val wifiPermissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { permissions ->
        if (permissions.values.any { !it }) {
            Toast.makeText(
                this,
                "未授予 Wi-Fi 权限，热力图层将不可用",
                Toast.LENGTH_LONG
            ).show()
        }
    }

    private val legacyStoragePermissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestPermission()
    ) { granted ->
        val source = pendingLegacyFloorPlanSave
        val mapSource = pendingLegacyMapExport
        pendingLegacyFloorPlanSave = null
        pendingLegacyMapExport = null
        if (granted) {
            if (mapSource?.exists() == true) {
                Thread {
                    try {
                        val location = writeMapToDownloads(mapSource)
                        runOnUiThread { show("地图已保存：$location") }
                    } catch (e: Exception) {
                        runOnUiThread { show("地图已保存在应用内，但导出失败：${e.message}") }
                    }
                }.start()
            }
            if (source?.exists() == true) saveFloorPlanFile(source)
        } else {
            show("未获得存储权限，无法保存到 Downloads/CartographerMaps")
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        tvPose = findViewById(R.id.tv_pose)
        tvDebugData = findViewById(R.id.tv_debug_data) // ✅ 绑定显示控件
        tvMapMeasurement = findViewById(R.id.tv_map_measurement)
        mapView = findViewById(R.id.map_view)
        floorPlanView = findViewById(R.id.iv_floor_plan)
        btnFinishMapping = findViewById(R.id.btn_finish_mapping)
        btnContinueMapping = findViewById(R.id.btn_continue_mapping)
        btnCancelContinueMapping = findViewById(R.id.btn_cancel_continue_mapping)
        btnSaveMap = findViewById(R.id.btn_save_map)
        btnSaveFloorPlan = findViewById(R.id.btn_save_floor_plan)
        btnMapNorth = findViewById(R.id.btn_map_north)
        checkPointCloud = findViewById(R.id.check_point_cloud)
        checkFloorPlanOverlay = findViewById(R.id.check_floor_plan_overlay)
        checkHeatMapOverlay = findViewById(R.id.check_heat_map_overlay)
        checkTrajectoryOverlay = findViewById(R.id.check_trajectory_overlay)

        lidarParser = LidarParser(carto)

        findViewById<Button>(R.id.btn_start_sensor).setOnClickListener {
            if (!isSensorRunning) {
                startSensors()
                startLidar()
                isSensorRunning = true
                Toast.makeText(this, "IMU + LiDAR 模式已开启", Toast.LENGTH_SHORT).show()
            }
        }

        findViewById<Button>(R.id.btn_start_mapping).setOnClickListener {
            if (!isSensorRunning) {
                Toast.makeText(this, "先开传感器！", Toast.LENGTH_SHORT).show()
                return@setOnClickListener
            }
            if (isMappingRunning || isFinishingMapping) {
                Toast.makeText(this, "当前正在建图", Toast.LENGTH_SHORT).show()
                return@setOnClickListener
            }
            if (isMappingFinished) {
                carto.reset()
            }
            requestWifiRssiPermissionsIfNeeded()
            mappingMode = MappingMode.NORMAL_MAPPING
            prepareMappingUi(clearMap = true)
            startCartographer(mapToLoad = null)
            Toast.makeText(this, "开始建图", Toast.LENGTH_SHORT).show()
        }

        btnContinueMapping.setOnClickListener {
            startContinueMapping()
        }

        btnFinishMapping.setOnClickListener {
            finishMapping()
        }
        btnCancelContinueMapping.setOnClickListener {
            cancelContinueMapping()
        }

        btnSaveMap.setOnClickListener {
            saveCurrentMap()
        }
        btnSaveFloorPlan.setOnClickListener {
            saveCurrentFloorPlan()
        }
        btnMapNorth.setOnClickListener {
            alignMapToMagneticNorth()
        }
        checkPointCloud.setOnCheckedChangeListener { _, _ ->
            refreshFloorPlanLayerPreview()
        }
        checkFloorPlanOverlay.setOnCheckedChangeListener { _, _ ->
            refreshFloorPlanLayerPreview()
        }
        checkHeatMapOverlay.setOnCheckedChangeListener { _, _ ->
            refreshFloorPlanLayerPreview()
        }
        checkTrajectoryOverlay.setOnCheckedChangeListener { _, _ ->
            refreshFloorPlanLayerPreview()
        }

        registerReceiver(usbReceiver, IntentFilter(ACTION_USB_PERMISSION), if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) Context.RECEIVER_NOT_EXPORTED else 0)
        startUIUpdate()
    }

    private fun startContinueMapping() {
        if (!isSensorRunning) {
            show("先开传感器！")
            return
        }
        if (isMappingRunning || isFinishingMapping) {
            show(if (isFinishingMapping) "正在完成优化，请稍候" else "当前正在建图")
            return
        }

        val mapDir = File(filesDir, "maps")
        val latestMap = mapDir.listFiles { file ->
            file.isFile && file.extension.equals("pbstream", ignoreCase = true)
        }?.maxByOrNull { it.lastModified() }
        if (latestMap == null) {
            show("没有找到可补采的旧地图")
            return
        }

        continueSourceMapFile = latestMap
        requestWifiRssiPermissionsIfNeeded()

        // A loaded state must go into a fresh MapBuilder. The loaded trajectories
        // are frozen; startCartographer creates a separate active trajectory later.
        setDataReceivePaused(true)
        carto.reset()
        mappingMode = MappingMode.CONTINUE_MAPPING
        relocalizationPhase = RelocalizationPhase.WAITING
        relocalizationStatus = RelocalizationStatus()
        prepareMappingUi(clearMap = false)
        startCartographer(latestMap)
        show("正在加载旧地图…")
    }

    private fun prepareMappingUi(clearMap: Boolean) {
        if (clearMap) {
            mapView.clearMap()
            clearMapMeasurement()
            continueBaseSubmaps = emptyList()
            continueSourceMapFile = null
            rssiSamples.clear()
            lastRssiSampleMs = 0L
        }
        clearFloorPlanLayers()
        mapView.resetToRobotFollowing()
        btnMapNorth.visibility = View.GONE
        btnMapNorth.alpha = 1f
        btnMapNorth.contentDescription = "磁力计：点击后将地图按磁北方向居中"
        isMagneticCalibrationRunning = false
        savedMapNorthAlignment = null
        savedMapFile = null
        pendingNorthAlignment = null
        loadedMapNorthAlignment = null
        finishedMapPose = null
        currentFloorPlanResult = null
        currentFloorPlanDimensions = null
        currentFloorPlanWarning = null
        pendingLegacyFloorPlanSave = null
        pendingLegacyMapExport = null
        lastRenderedScanNs = 0L
        lastSubmapFetchMs = 0L
        lastFetchedSubmapNodeCount = -1L
        lastFullSubmapFetchMs = 0L
        lastRelocalizationStatusFetchMs = 0L
        isRelocalizationStatusFetchRunning = false
        latestSubmapRefreshIntervalMs = 0L
        lastObservedRangeFrames = 0L
        lastObservedLocalSlamResults = 0L
        lastObservedInsertedNodes = 0L
        lastRangeFrameChangeMs = 0L
        lastLocalSlamChangeMs = 0L
        lastInsertedNodeChangeMs = 0L
        latestRangeFrameIntervalMs = 0L
        latestLocalSlamIntervalMs = 0L
        latestInsertedNodeIntervalMs = 0L
        latestSubmapCount = 0
        if (mappingMode == MappingMode.NORMAL_MAPPING) {
            relocalizationPhase = RelocalizationPhase.NOT_REQUIRED
            relocalizationStatus = RelocalizationStatus()
        }
        isMappingFinished = false
        isMappingRunning = true
        btnFinishMapping.isEnabled = false
        btnContinueMapping.isEnabled = false
        btnSaveMap.isEnabled = false
        btnSaveFloorPlan.isEnabled = false
        btnCancelContinueMapping.visibility = if (mappingMode == MappingMode.CONTINUE_MAPPING) {
            View.VISIBLE
        } else {
            View.GONE
        }
    }

    private fun startCartographer(mapToLoad: File?) {
        setDataReceivePaused(true)

        Thread {
            try {
                val localConfigDir = File(filesDir, "config")
                prepareCartographerConfig(localConfigDir)

                val configurationFile = if (mapToLoad == null) {
                    "android_2d.lua"
                } else {
                    "android_2d_continue.lua"
                }
                val ok = carto.initialize(
                    localConfigDir.absolutePath,
                    configurationFile
                )
                if (!ok) {
                    handleMappingStartFailure("初始化失败")
                    return@Thread
                }

                if (mapToLoad != null) {
                    val loaded = carto.loadMap(mapToLoad.absolutePath, loadFrozenState = true)
                    if (!loaded) {
                        handleMappingStartFailure("旧地图加载失败")
                        return@Thread
                    }
                    val oldSubmaps = carto.getSubmapTextures()
                    loadedMapNorthAlignment = MapMetadataStore.load(mapToLoad)
                    pendingNorthAlignment = loadedMapNorthAlignment
                    runOnUiThread {
                        continueBaseSubmaps = oldSubmaps
                        latestSubmapCount = oldSubmaps.size
                        mapView.setSubmapTextures(oldSubmaps)
                        // Map measurement/floor-plan geometry is not needed for
                        // relocalization and can be expensive on a large frozen
                        // map. Recompute it once matching succeeds.
                    }
                }

                val traj = carto.startTrajectory()
                if (traj < 0) {
                    handleMappingStartFailure("轨迹启动失败")
                    return@Thread
                }

                setDataReceivePaused(false)
                if (mapToLoad != null) {
                    relocalizationStartedMs = SystemClock.elapsedRealtime()
                }
                runOnUiThread {
                    btnFinishMapping.isEnabled = mapToLoad == null
                    show(if (mapToLoad == null) {
                        "✅ 建图启动成功"
                    } else {
                        "旧地图已加载，正在重定位；请在已建区域缓慢移动和转动"
                    })
                }

            } catch (e: Exception) {
                e.printStackTrace()
                handleMappingStartFailure("错误：${e.message}")
            }
        }.start()
    }

    private fun handleMappingStartFailure(message: String) {
        setDataReceivePaused(true)
        isMappingRunning = false
        carto.reset()
        runOnUiThread {
            btnContinueMapping.isEnabled = true
            btnCancelContinueMapping.visibility = View.GONE
            show(message)
        }
    }

    private fun prepareCartographerConfig(outputDir: File) {
        if (!outputDir.exists()) outputDir.mkdirs()
        val plainAssets = assets.list("config").orEmpty().filter { it.endsWith(".lua") }
        if (plainAssets.isNotEmpty()) {
            plainAssets.forEach { name ->
                assets.open("config/$name").use { input ->
                    FileOutputStream(File(outputDir, name), false).use { output -> input.copyTo(output) }
                }
            }
            return
        }

        // Binary SDK deliveries carry obfuscated configuration resources rather
        // than plaintext Lua. Decrypt only into the app-private files directory.
        val key = byteArrayOf(0x43, 0x61, 0x72, 0x74, 0x6f, 0x53, 0x44, 0x4b)
        val protectedAssets = assets.list("carto_cfg").orEmpty().filter { it.endsWith(".bin") }
        require(protectedAssets.isNotEmpty()) { "Cartographer configuration is missing" }
        protectedAssets.forEach { name ->
            val bytes = assets.open("carto_cfg/$name").use { it.readBytes() }
            for (index in bytes.indices) {
                bytes[index] = (bytes[index].toInt() xor key[index % key.size].toInt()).toByte()
            }
            File(outputDir, name.removeSuffix(".bin")).writeBytes(bytes)
        }
    }

    @Volatile private var pauseDataReceive = false

    private fun setDataReceivePaused(paused: Boolean) {
        pauseDataReceive = paused
        if (!paused) return
        lidarParser.clearLatestScanAndPendingData()
        lastRenderedScanNs = 0L
        runOnUiThread { mapView.clearLiveScan() }
    }

    private fun startSensors() {
        // IMU 模式：
        val a = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        val g = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        sensorManager.registerListener(this, a, SensorManager.SENSOR_DELAY_FASTEST)
        sensorManager.registerListener(this, g, SensorManager.SENSOR_DELAY_FASTEST)
        lidarParser.enableLidarAfterImuReady()
        if (!magneticHeadingProvider.start()) {
            show("当前设备没有可用磁力计，建图不受影响，但无法进行地图南北对齐")
        } else {
            val magneticState = magneticHeadingProvider.getState()
            Log.i(
                "MagneticHeading",
                "磁力计已注册：${magneticState.sensorName}, " +
                    "vendor=${magneticState.sensorVendor}"
            )
        }

        // 纯 LiDAR 测试模式：不注册手机加速度计和陀螺仪。
        // hasAccelerationSample = false
        // hasGyroscopeSample = false
        // imuCount = 0
        // lidarParser.enableLidar()
    }

    private fun startLidar() {
        val drivers = UsbSerialProber.getDefaultProber().findAllDrivers(usbManager)
        if (drivers.isEmpty()) return
        val d = drivers[0]
        if (!usbManager.hasPermission(d.device)) {
            val f = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) PendingIntent.FLAG_MUTABLE else 0
            val pi = PendingIntent.getBroadcast(this, 0, Intent(ACTION_USB_PERMISSION), f)
            usbManager.requestPermission(d.device, pi)
        } else {
            openPort(d.ports[0])
        }
    }

    private fun openPort(port: UsbSerialPort) {
        try {
            port.open(usbManager.openDevice(port.device))
            port.setParameters(230400, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE)
            port.dtr = false
            usbSerialPort = port
            Thread.sleep(200)
            sendLidarCommand(port, 0xF0, 0x0F)

            usbIoManager = SerialInputOutputManager(port, object : SerialInputOutputManager.Listener {
                override fun onNewData(data: ByteArray) {
                    lidarCount += data.size
                    if (pauseDataReceive) return

                    // ✅ 不加 isMappingRunning 判断！不点建图也能读雷达！
                    try {
                        lidarParser.onDataReceived(data, data.size)

                        // ✅ 实时读取雷达数据（测试用）
                        val lastPoints = lidarParser.lastValidPointCount
                        val lastDist = lidarParser.lastShowDistance
                        val lastAngle = lidarParser.lastShowAngle
                        runOnUiThread {
                            latestLidarPoints = lastPoints
                            latestLidarDist = lastDist
                            latestLidarAngle = lastAngle
                        }

                    } catch (e: Exception) {
                        e.printStackTrace()
                    }
                }
                override fun onRunError(e: Exception) {}
            }).apply { start() }

        } catch (e: Exception) {
            e.printStackTrace()
        }
    }

    override fun onSensorChanged(e: SensorEvent) {
        if (!isSensorRunning) return

        when (e.sensor.type) {
            Sensor.TYPE_ACCELEROMETER -> {
                acc[0] = e.values[0]
                acc[1] = e.values[1]
                acc[2] = e.values[2]
                lastAccelerationSampleNs = e.timestamp
                hasAccelerationSample = true
            }
            Sensor.TYPE_GYROSCOPE -> {
                gyro[0] = e.values[0]
                gyro[1] = e.values[1]
                gyro[2] = e.values[2]
                hasGyroscopeSample = true
            }
            else -> return
        }

        // SensorEvent.timestamp is the hardware sample time on the same
        // CLOCK_BOOTTIME time base as elapsedRealtimeNanos().
        if (e.sensor.type != Sensor.TYPE_GYROSCOPE ||
            !hasAccelerationSample ||
            !hasGyroscopeSample ||
            e.timestamp - lastAccelerationSampleNs !in
                0L..IMU_MAX_PAIR_AGE_NS ||
            e.timestamp - lastImuSendNs < IMU_MIN_INTERVAL_NS) {
            return
        }
        lastImuSendNs = e.timestamp
        lastImuTimeNs = e.timestamp
        carto.addImuData(e.timestamp, acc, gyro)
        imuCount++
    }



    private fun startUIUpdate() {
        val h = Handler(Looper.getMainLooper())
        h.post(object : Runnable {
            override fun run() {
                val pose = carto.getCurrentPose()
                val status = carto.getStatus()
                val magneticState = magneticHeadingProvider.getState()
                val nowMs = SystemClock.elapsedRealtime()
                updateSlamTiming(status, nowMs)
                val pose2d = pose?.to2D()
                maybeSampleWifiRssi(pose2d, nowMs)
                val latestScan = lidarParser.getLatestScan()
                val scanAgeNs = latestScan?.let {
                    SystemClock.elapsedRealtimeNanos() - it.timestampNs
                }
                val hasFreshScan = latestScan != null && scanAgeNs != null &&
                    scanAgeNs in 0..LIVE_SCAN_STALE_TIMEOUT_NS
                if (!pauseDataReceive && hasFreshScan &&
                    latestScan!!.timestampNs != lastRenderedScanNs) {
                    mapView.setLiveScan(latestScan, pose2d)
                    lastRenderedScanNs = latestScan.timestampNs
                } else if ((pauseDataReceive || !hasFreshScan) && lastRenderedScanNs != 0L) {
                    mapView.clearLiveScan()
                    lastRenderedScanNs = 0L
                }
                maybeFetchSubmaps(status.insertedNodes)
                val tip = when {
                    isFinishingMapping -> "正在完成优化"
                    isMappingRunning && relocalizationPhase == RelocalizationPhase.WAITING -> "正在旧地图中重定位"
                    isMappingRunning && relocalizationPhase == RelocalizationPhase.TIMEOUT -> "重定位尚未成功，请回到已建区域"
                    isMappingRunning && relocalizationPhase == RelocalizationPhase.RELOCALIZED -> "已对齐，补采建图中"
                    isMappingRunning -> "建图中"
                    isMappingFinished -> "建图已结束"
                    isSensorRunning -> "IMU + LiDAR 已开启"
                    else -> "待机"
                }
                tvPose.text = if (pose2d != null) {
                    "$tip\nIMU:$imuCount 雷达:$lidarCount\nX:%.2f Y:%.2f θ:%.1f°".format(pose2d.x, pose2d.y, Math.toDegrees(pose2d.theta))
                } else {
                    "$tip\nIMU:$imuCount 雷达:$lidarCount"
                }

                // ===============================
                // ✅ 实时显示真实数据到屏幕！
                // ===============================
                if (nowMs - lastDebugTextUpdateMs >= DEBUG_TEXT_UPDATE_INTERVAL_MS) {
                    lastDebugTextUpdateMs = nowMs
                    val debugText = """
                    📱 实时传感器数据
                    ─────────────────────
                    IMU：已启用（加速度计 + 陀螺仪）

                    磁力计：
                    设备：${magneticState.sensorName}
                    厂商：${magneticState.sensorVendor}
                    支持/注册：${if (magneticState.isSupported) "是" else "否"}/${if (magneticState.isRegistered) "是" else "否"}
                    回调：${magneticState.callbackCount}
                    X/Y/Z：${String.format(Locale.US, "%.1f / %.1f / %.1f μT", magneticState.magneticXMicroTesla, magneticState.magneticYMicroTesla, magneticState.magneticZMicroTesla)}
                    磁场强度：${String.format(Locale.US, "%.1f μT", magneticState.fieldStrengthMicroTesla)}，波动：${String.format(Locale.US, "%.1f μT", magneticState.fieldStrengthStdDevMicroTesla)}
                    系统精度：${formatMagneticAccuracy(magneticState.accuracy)}
                    航向样本：${magneticState.sampleCount}，时长：${String.format(Locale.US, "%.1f s", magneticState.samplingDurationMillis / 1000.0)}，波动：${formatMagneticDispersion(magneticState.dispersionDegrees)}
                    校准状态：${if (magneticState.isStable) "稳定" else "等待稳定"}
                    
                    雷达：
                    有效点数：$latestLidarPoints
                    距离：${String.format("%.2f", latestLidarDist)} m
                    角度：${String.format("%.1f", latestLidarAngle)} °
                    协议：${lidarParser.activeProtocol}
                    AA55包：${lidarParser.aa55PacketCount}
                    异常包：${lidarParser.invalidAa55PacketCount}
                    残缺转圈：${lidarParser.droppedIncompleteScanCount}
                    丢弃字节：${lidarParser.droppedByteCount}
                    离群点：${lidarParser.rejectedOutlierPointCount}

                    Cartographer：
                    雷达帧：${status.rangeFrames}
                    IMU：${status.imuSamples} 帧
                    LocalSLAM：${status.localSlamResults}
                    插入节点：${status.insertedNodes}
                    Submap：$latestSubmapCount
                    位姿：${if (status.hasPose) "有" else "无"}
                    ${formatRelocalizationDebug()}

                    实际间隔：
                    雷达帧：${formatInterval(latestRangeFrameIntervalMs)}
                    LocalSLAM：${formatInterval(latestLocalSlamIntervalMs)}
                    插入节点：${formatInterval(latestInsertedNodeIntervalMs)}
                    UI刷图：${formatInterval(latestSubmapRefreshIntervalMs)}
                    """.trimIndent()
                    tvDebugData.text = debugText
                }

                h.postDelayed(this, 100)
            }
        })
    }

    private fun updateSlamTiming(status: SlamStatus, nowMs: Long) {
        if (status.rangeFrames != lastObservedRangeFrames) {
            if (lastRangeFrameChangeMs > 0L) {
                latestRangeFrameIntervalMs = nowMs - lastRangeFrameChangeMs
            }
            lastRangeFrameChangeMs = nowMs
            lastObservedRangeFrames = status.rangeFrames
        }
        if (status.localSlamResults != lastObservedLocalSlamResults) {
            if (lastLocalSlamChangeMs > 0L) {
                latestLocalSlamIntervalMs = nowMs - lastLocalSlamChangeMs
            }
            lastLocalSlamChangeMs = nowMs
            lastObservedLocalSlamResults = status.localSlamResults
        }
        if (status.insertedNodes != lastObservedInsertedNodes) {
            if (lastInsertedNodeChangeMs > 0L) {
                latestInsertedNodeIntervalMs = nowMs - lastInsertedNodeChangeMs
            }
            lastInsertedNodeChangeMs = nowMs
            lastObservedInsertedNodes = status.insertedNodes
        }
    }

    private fun requestWifiRssiPermissionsIfNeeded() {
        val missing = buildList {
            if (ContextCompat.checkSelfPermission(
                    this@MainActivity,
                    Manifest.permission.ACCESS_FINE_LOCATION
                ) != PackageManager.PERMISSION_GRANTED) {
                add(Manifest.permission.ACCESS_FINE_LOCATION)
            }
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU &&
                ContextCompat.checkSelfPermission(
                    this@MainActivity,
                    Manifest.permission.NEARBY_WIFI_DEVICES
                ) != PackageManager.PERMISSION_GRANTED) {
                add(Manifest.permission.NEARBY_WIFI_DEVICES)
            }
        }
        if (missing.isNotEmpty()) wifiPermissionLauncher.launch(missing.toTypedArray())
    }

    private fun hasWifiRssiPermissions(): Boolean {
        val hasLocation = ContextCompat.checkSelfPermission(
            this,
            Manifest.permission.ACCESS_FINE_LOCATION
        ) == PackageManager.PERMISSION_GRANTED
        val hasNearby = Build.VERSION.SDK_INT < Build.VERSION_CODES.TIRAMISU ||
            ContextCompat.checkSelfPermission(
                this,
                Manifest.permission.NEARBY_WIFI_DEVICES
            ) == PackageManager.PERMISSION_GRANTED
        return hasLocation && hasNearby
    }

    @Suppress("DEPRECATION")
    private fun maybeSampleWifiRssi(pose: Pose2D?, nowMs: Long) {
        if (!isMappingRunning || pose == null ||
            nowMs - lastRssiSampleMs < RSSI_SAMPLE_INTERVAL_MS ||
            !hasWifiRssiPermissions()) return

        val rssi = try {
            wifiManager.connectionInfo?.rssi ?: return
        } catch (_: SecurityException) {
            return
        }
        if (rssi !in -126..-1) return
        rssiSamples += RssiSample(
            worldX = pose.x.toFloat(),
            worldY = pose.y.toFloat(),
            rssiDbm = rssi.toFloat(),
            timestampMillis = System.currentTimeMillis()
        )
        lastRssiSampleMs = nowMs
    }

    private fun formatInterval(intervalMs: Long): String {
        return if (intervalMs <= 0L) "--" else "${intervalMs}ms"
    }

    private fun formatRelocalizationDebug(): String {
        if (mappingMode != MappingMode.CONTINUE_MAPPING || !isMappingRunning) return ""
        val stateText = when (relocalizationPhase) {
            RelocalizationPhase.WAITING -> "等待匹配"
            RelocalizationPhase.RELOCALIZED -> "已成功"
            RelocalizationPhase.TIMEOUT -> "尚未成功"
            RelocalizationPhase.NOT_REQUIRED -> "--"
        }
        return """
            重定位：$stateText
            新轨迹节点：${relocalizationStatus.activeNodeCount}
            跨轨迹约束：${relocalizationStatus.interTrajectoryConstraintCount}
            已匹配新节点：${relocalizationStatus.matchedActiveNodeCount}
            已匹配旧Submap：${relocalizationStatus.matchedOldSubmapCount}
        """.trimIndent()
    }

    private fun maybeFetchSubmaps(insertedNodeCount: Long) {
        if (!isMappingRunning || isFinishingMapping) return
        if (mappingMode == MappingMode.CONTINUE_MAPPING &&
            relocalizationPhase != RelocalizationPhase.RELOCALIZED) {
            maybeUpdateRelocalizationStatus()
            return
        }
        if (isSubmapFetchRunning) return
        val nowMs = SystemClock.elapsedRealtime()
        val fullRefreshDue = lastFullSubmapFetchMs > 0L &&
            nowMs - lastFullSubmapFetchMs >= FULL_SUBMAP_FETCH_INTERVAL_MS
        if (insertedNodeCount <= lastFetchedSubmapNodeCount && !fullRefreshDue) return
        if (nowMs - lastSubmapFetchMs < SUBMAP_FETCH_INTERVAL_MS) return
        val previousFetchMs = lastSubmapFetchMs
        lastSubmapFetchMs = nowMs
        val fetchAllSubmaps = lastFullSubmapFetchMs == 0L || fullRefreshDue
        isSubmapFetchRunning = true

        Thread {
            try {
                // Most refreshes only decode the active submaps. Decoding and
                // converting every historical submap while holding Cartographer's
                // data mutex made sensor insertion progressively slower. A low
                // frequency full refresh still applies pose-graph corrections to
                // cached historical submaps. Final optimization/export always
                // requests the complete set independently of this display path.
                val textures = if (fetchAllSubmaps) {
                    carto.getSubmapTextures()
                } else {
                    // Cartographer inserts into two overlapping active submaps.
                    // Fetching only the newest one leaves half of the live map
                    // stale until the next full refresh.
                    carto.getActiveSubmapTextures()
                }
                if (textures.isNotEmpty()) {
                    lastFetchedSubmapNodeCount = maxOf(
                        lastFetchedSubmapNodeCount,
                        insertedNodeCount
                    )
                    if (fetchAllSubmaps) lastFullSubmapFetchMs = nowMs
                }
                if (previousFetchMs > 0L) {
                    latestSubmapRefreshIntervalMs = nowMs - previousFetchMs
                }
                if (textures.isNotEmpty()) {
                    latestSubmapCount = if (fetchAllSubmaps) {
                        textures.size
                    } else {
                        maxOf(
                            latestSubmapCount,
                            textures.maxOf { it.submapIndex + 1 }
                        )
                    }
                }
                if (textures.isNotEmpty()) {
                    runOnUiThread {
                        mapView.setSubmapTextures(textures)
                        // MapMeasurementCalculator scans all cached texture
                        // pixels. Re-running it on every 500 ms live texture
                        // update can consume more CPU than the renderer itself.
                        if (fetchAllSubmaps &&
                            (mappingMode == MappingMode.NORMAL_MAPPING ||
                                relocalizationPhase == RelocalizationPhase.RELOCALIZED)) {
                            mergeMeasurementTextures(textures)
                        }
                    }
                }
            } catch (e: Exception) {
                Log.e("CartographerJNI", "获取Submap失败", e)
            } finally {
                isSubmapFetchRunning = false
            }
        }.start()
    }

    private fun maybeUpdateRelocalizationStatus() {
        if (mappingMode != MappingMode.CONTINUE_MAPPING || !isMappingRunning ||
            isFinishingMapping || isRelocalizationStatusFetchRunning ||
            relocalizationPhase == RelocalizationPhase.RELOCALIZED) {
            return
        }
        val nowMs = SystemClock.elapsedRealtime()
        if (nowMs - lastRelocalizationStatusFetchMs <
            RELOCALIZATION_STATUS_INTERVAL_MS) {
            return
        }
        lastRelocalizationStatusFetchMs = nowMs
        isRelocalizationStatusFetchRunning = true
        Thread {
            try {
                val newStatus = carto.getRelocalizationStatus()
                relocalizationStatus = newStatus
                val previousPhase = relocalizationPhase
                relocalizationPhase = when {
                    newStatus.isRelocalized -> RelocalizationPhase.RELOCALIZED
                    SystemClock.elapsedRealtime() - relocalizationStartedMs >=
                        RELOCALIZATION_TIMEOUT_MS ||
                        newStatus.activeNodeCount >= RELOCALIZATION_TIMEOUT_NODE_COUNT ->
                        RelocalizationPhase.TIMEOUT
                    else -> RelocalizationPhase.WAITING
                }
                if (previousPhase != RelocalizationPhase.RELOCALIZED &&
                    relocalizationPhase == RelocalizationPhase.RELOCALIZED) {
                    // Only after matching succeeds do the expensive full texture
                    // extraction and fused-map rendering. During global search the
                    // static loaded map remains visible and CPU stays with SLAM.
                    val alignedSubmaps = carto.getSubmapTextures()
                    latestSubmapCount = alignedSubmaps.size
                    val alignedFetchMs = SystemClock.elapsedRealtime()
                    lastSubmapFetchMs = alignedFetchMs
                    lastFullSubmapFetchMs = alignedFetchMs
                    runOnUiThread {
                        mapView.setSubmapTextures(alignedSubmaps)
                        replaceMeasurementTextures(alignedSubmaps)
                        btnFinishMapping.isEnabled = true
                        show("✅ 已与旧地图建立稳定约束，可以开始正式补采")
                    }
                }
            } catch (e: Exception) {
                Log.e("CartographerJNI", "查询重定位状态失败", e)
            } finally {
                isRelocalizationStatusFetchRunning = false
            }
        }.start()
    }

    private fun finishMapping() {
        if (!isMappingRunning || isFinishingMapping) {
            Toast.makeText(this, "当前没有正在运行的建图轨迹", Toast.LENGTH_SHORT).show()
            return
        }
        if (mappingMode == MappingMode.CONTINUE_MAPPING &&
            relocalizationPhase != RelocalizationPhase.RELOCALIZED) {
            show("补采轨迹尚未与旧地图对齐，请继续在已建区域缓慢移动；不需要本次数据可点取消补采")
            return
        }

        isFinishingMapping = true
        magneticHeadingProvider.beginCalibration()
        setDataReceivePaused(true)
        btnFinishMapping.isEnabled = false
        btnSaveMap.isEnabled = false
        show("正在结束建图并完成优化…")

        Thread {
            try {
                carto.finishTrajectory()
                val finalPose = carto.getLastFinishedPose()?.to2D()
                finishedMapPose = finalPose
                // Lock magnetic heading while the trajectory is ending, when the
                // optimized endpoint still represents the handheld device direction.
                // A loaded map keeps its original north reference.
                val finishHeading = magneticHeadingProvider.getState()
                pendingNorthAlignment = loadedMapNorthAlignment
                    ?: calculateMapNorthAlignment(finishHeading, finalPose)
                val optimizedSubmaps = carto.getSubmapTextures()
                val optimizedTrajectory = carto.getTrajectoryNodePoses()
                isMappingRunning = false
                isMappingFinished = true
                runOnUiThread {
                    latestSubmapCount = optimizedSubmaps.size
                    mapView.setSubmapTextures(optimizedSubmaps)
                    replaceMeasurementTextures(optimizedSubmaps)
                    mapView.setOptimizedTrajectory(optimizedTrajectory)
                    btnSaveMap.isEnabled = true
                    btnContinueMapping.isEnabled = true
                    btnCancelContinueMapping.visibility = View.GONE
                    show(if (mappingMode == MappingMode.CONTINUE_MAPPING) {
                        northAlignmentFinishMessage("补采与优化已完成，可以保存扩展地图")
                    } else {
                        northAlignmentFinishMessage("建图与优化已完成，可以保存地图")
                    })
                }
            } catch (e: Exception) {
                Log.e("CartographerJNI", "结束建图失败", e)
                setDataReceivePaused(false)
                runOnUiThread {
                    btnFinishMapping.isEnabled = true
                    show("结束建图失败：${e.message}")
                }
            } finally {
                isFinishingMapping = false
            }
        }.start()
    }

    private fun saveCurrentMap() {
        if (!isMappingFinished) {
            Toast.makeText(this, "请先结束建图", Toast.LENGTH_SHORT).show()
            return
        }
        if (mappingMode == MappingMode.CONTINUE_MAPPING &&
            relocalizationPhase != RelocalizationPhase.RELOCALIZED) {
            Toast.makeText(this, "补采轨迹未与旧地图对齐，禁止保存扩展地图", Toast.LENGTH_LONG).show()
            return
        }

        btnSaveMap.isEnabled = false
        Thread {
            val mapDir = File(filesDir, "maps")
            if (!mapDir.exists()) mapDir.mkdirs()
            val timestamp = System.currentTimeMillis()
            val prefix = if (mappingMode == MappingMode.CONTINUE_MAPPING) {
                "cartographer_extended"
            } else {
                "cartographer"
            }
            val output = File(mapDir, "${prefix}_$timestamp.pbstream")
            val ok = try {
                carto.saveMap(output.absolutePath)
            } catch (e: Exception) {
                Log.e("CartographerJNI", "保存地图失败", e)
                false
            }
            val alignmentForSave = pendingNorthAlignment
            val metadataSaved = ok && alignmentForSave != null &&
                MapMetadataStore.save(output, alignmentForSave)
            var publicMapLocation: String? = null
            var publicMapExportError: String? = null
            if (ok) {
                if (Build.VERSION.SDK_INT <= Build.VERSION_CODES.P &&
                    ContextCompat.checkSelfPermission(
                        this,
                        Manifest.permission.WRITE_EXTERNAL_STORAGE
                    ) != PackageManager.PERMISSION_GRANTED) {
                    pendingLegacyMapExport = output
                    runOnUiThread {
                        legacyStoragePermissionLauncher.launch(
                            Manifest.permission.WRITE_EXTERNAL_STORAGE
                        )
                    }
                } else {
                    try {
                        publicMapLocation = writeMapToDownloads(output)
                    } catch (e: Exception) {
                        Log.e("CartographerJNI", "导出地图到 Downloads 失败", e)
                        publicMapExportError = e.message ?: "未知错误"
                    }
                }
                runOnUiThread {
                    savedMapFile = output
                    savedMapNorthAlignment = alignmentForSave
                    btnMapNorth.visibility = View.VISIBLE
                    btnMapNorth.isEnabled = true
                    btnMapNorth.alpha = 1f
                    btnMapNorth.contentDescription = "磁力计：点击后将地图按磁北方向居中"
                    btnMapNorth.bringToFront()
                }
            }

            val finalizedTextures = try {
                carto.getSubmapTextures()
            } catch (e: Exception) {
                Log.e("CartographerJNI", "获取最终融合地图失败", e)
                emptyList()
            }
            if (finalizedTextures.isNotEmpty()) {
                // Switch the main map as soon as the saved submaps are ready.
                // Floor-plan reconstruction can take noticeably longer and
                // must not delay the finalized black occupancy presentation.
                runOnUiThread {
                    mapView.setFinalizedSubmapTextures(finalizedTextures)
                }
            }

            val floorPlanOutput = try {
                exportFloorPlanMaps(timestamp, finalizedTextures)
            } catch (e: Exception) {
                Log.e("CartographerJNI", "导出户型图输入地图失败", e)
                null
            }

            val floorPlanResult = try {
                floorPlanOutput?.let { generateFloorPlan(timestamp, it) }
            } catch (e: Exception) {
                Log.e("CartographerJNI", "生成户型图失败", e)
                null
            }

            runOnUiThread {
                btnSaveMap.isEnabled = true
                val floorPlanText = updateCurrentFloorPlan(floorPlanOutput, floorPlanResult)
                val mapText = if (ok) {
                    when {
                        publicMapLocation != null -> "地图已保存：$publicMapLocation"
                        publicMapExportError != null ->
                            "地图已保存在应用内：${output.absolutePath}\n" +
                                "导出到 Downloads/CartographerMaps 失败：$publicMapExportError"
                        Build.VERSION.SDK_INT <= Build.VERSION_CODES.P ->
                            "地图已保存在应用内，授权后将导出到 Downloads/CartographerMaps"
                        else -> "地图已保存：${output.absolutePath}"
                    }
                } else {
                    "地图保存失败"
                }
                val northText = if (!ok) {
                    ""
                } else when {
                    alignmentForSave == null -> "\n磁北方向：点击地图上的南北按钮进行校准"
                    metadataSaved -> "\n磁北方向信息：已保存"
                    else -> "\n地图已保存，但磁北方向元数据保存失败"
                }
                show("$mapText\n$floorPlanText$northText")
            }
        }.start()
    }

    private fun cancelContinueMapping() {
        if (!isMappingRunning || mappingMode != MappingMode.CONTINUE_MAPPING || isFinishingMapping) {
            show("当前没有可取消的补采任务")
            return
        }
        isFinishingMapping = true
        setDataReceivePaused(true)
        btnFinishMapping.isEnabled = false
        btnCancelContinueMapping.isEnabled = false
        Thread {
            try {
                carto.finishTrajectory()
                finishedMapPose = carto.getLastFinishedPose()?.to2D()
            } catch (e: Exception) {
                Log.w("CartographerJNI", "取消补采时结束轨迹失败", e)
            } finally {
                carto.reset()
                isMappingRunning = false
                isMappingFinished = false
                isFinishingMapping = false
                relocalizationPhase = RelocalizationPhase.NOT_REQUIRED
                runOnUiThread {
                    mapView.clearMap()
                    mapView.setSubmapTextures(continueBaseSubmaps)
                    replaceMeasurementTextures(continueBaseSubmaps)
                    savedMapFile = continueSourceMapFile
                    savedMapNorthAlignment = loadedMapNorthAlignment
                    btnMapNorth.visibility = if (savedMapFile?.exists() == true) View.VISIBLE else View.GONE
                    btnMapNorth.isEnabled = true
                    btnMapNorth.alpha = 1f
                    btnMapNorth.contentDescription = "磁力计：点击后将地图按磁北方向居中"
                    btnMapNorth.bringToFront()
                    btnContinueMapping.isEnabled = true
                    btnCancelContinueMapping.isEnabled = true
                    btnCancelContinueMapping.visibility = View.GONE
                    show("已取消本次补采，未保存未对齐轨迹")
                }
            }
        }.start()
    }

    private fun calculateMapNorthAlignment(
        heading: MagneticHeadingState,
        finalPose: Pose2D?
    ): MapNorthAlignment? {
        if (!heading.isStable || finalPose == null) return null
        // Android azimuth is clockwise from magnetic north. Therefore magnetic
        // north is headingRadians counter-clockwise from the robot forward
        // direction. Add that relative angle to the optimized map yaw.
        val northYaw = normalizeRadians(
            finalPose.theta + heading.headingRadians + DEVICE_TO_ROBOT_HEADING_OFFSET_RAD
        )
        return MapNorthAlignment(
            magneticNorthYawRadians = northYaw.toFloat(),
            headingAccuracy = heading.accuracy,
            sampleTimestampNs = heading.timestampNs
        )
    }

    private fun northAlignmentFinishMessage(baseMessage: String): String {
        return if (pendingNorthAlignment != null) {
            "$baseMessage；磁北方向已锁定"
        } else {
            "$baseMessage；保存后点击地图右上角磁力计图标，短暂保持朝向完成磁北校准"
        }
    }

    private fun alignMapToMagneticNorth() {
        val mapFile = savedMapFile
        val alignment = savedMapNorthAlignment
        if (mapFile == null || !mapFile.exists()) {
            show("请先完成并保存地图")
            return
        }
        if (alignment == null) {
            startMagneticCalibration()
            return
        }
        val measurement = currentMapMeasurement
        if (measurement == null) {
            show("地图尺寸仍在计算，暂时无法自动居中")
            return
        }
        if (mapView.showMagneticNorthOverview(
                measurement,
                alignment.magneticNorthYawRadians
            )) {
            show("地图已按磁北方向居中：左侧为南，右侧为北")
        } else {
            show("地图范围不足，无法自动居中")
        }
    }

    private fun startMagneticCalibration() {
        if (isMagneticCalibrationRunning) {
            show("磁北校准正在进行，请保持设备静止")
            return
        }
        val mapFile = savedMapFile
        if (mapFile == null || !mapFile.exists()) {
            show("请先完成并保存地图")
            return
        }
        val finalPose = finishedMapPose
        if (finalPose == null) {
            show("没有最终地图位姿，无法计算地图北向")
            return
        }
        if (!magneticHeadingProvider.isSupported || !magneticHeadingProvider.start()) {
            show("当前手机没有可用的磁力传感器")
            return
        }

        magneticHeadingProvider.beginCalibration()
        isMagneticCalibrationRunning = true
        btnMapNorth.isEnabled = false
        btnMapNorth.alpha = 0.55f
        btnMapNorth.contentDescription = "磁力计正在校准，请保持设备静止"
        show("请自然手持并短暂保持朝向，正在采集磁力计数据…")

        Thread {
            val deadline = SystemClock.elapsedRealtime() + MAGNETIC_CALIBRATION_TIMEOUT_MS
            var state = magneticHeadingProvider.getState()
            while (isMagneticCalibrationRunning &&
                SystemClock.elapsedRealtime() < deadline && !state.isStable) {
                try {
                    Thread.sleep(200L)
                } catch (_: InterruptedException) {
                    break
                }
                state = magneticHeadingProvider.getState()
            }
            if (!isMagneticCalibrationRunning) return@Thread
            val alignment = calculateMapNorthAlignment(state, finalPose)
            val metadataSaved = alignment?.let { MapMetadataStore.save(mapFile, it) } ?: false
            runOnUiThread {
                isMagneticCalibrationRunning = false
                btnMapNorth.isEnabled = true
                btnMapNorth.alpha = 1f
                btnMapNorth.contentDescription = "磁力计：点击后将地图按磁北方向居中"
                if (alignment == null) {
                    show(magneticCalibrationFailureMessage(state))
                    return@runOnUiThread
                }
                pendingNorthAlignment = alignment
                savedMapNorthAlignment = alignment
                val accuracyWarning = if (
                    state.accuracy == SensorManager.SENSOR_STATUS_UNRELIABLE
                ) "（系统精度较低，建议远离电机和金属后复核）" else ""
                show(if (metadataSaved) {
                    "磁北校准成功$accuracyWarning"
                } else {
                    "磁北校准成功，但方向元数据保存失败$accuracyWarning"
                })
                alignMapToMagneticNorth()
            }
        }.start()
    }

    private fun magneticCalibrationFailureMessage(state: MagneticHeadingState): String {
        return when {
            !state.isSupported -> "当前手机没有加速度计或磁力传感器"
            !state.isRegistered -> "磁力传感器注册失败，请重新开启传感器后重试"
            state.callbackCount <= 0L ->
                "没有收到手机磁力传感器回调，请检查设备传感器是否可用"
            state.sampleCount < 25 || state.samplingDurationMillis < 1_500L ->
                "磁力计连续有效样本不足（${state.sampleCount}/25，${
                    String.format(Locale.US, "%.1f", state.samplingDurationMillis / 1000.0)
                }/1.5 秒），请短暂保持朝向后重试"
            state.fieldStrengthMicroTesla !in 15f..100f ->
                "磁场强度异常（${String.format(Locale.US, "%.1f", state.fieldStrengthMicroTesla)} μT），" +
                    "请远离磁铁、雷达电机、金属和大电流线缆后重试"
            state.fieldStrengthStdDevMicroTesla > 6f ->
                "磁场波动过大（${String.format(Locale.US, "%.1f", state.fieldStrengthStdDevMicroTesla)} μT），" +
                    "请远离雷达电机和大电流线缆，静置后重试"
            state.dispersionDegrees > 12f ->
                "磁力航向波动过大（${String.format(Locale.US, "%.1f", state.dispersionDegrees)}°），" +
                    "请短暂保持朝向后重试"
            else -> "磁北校准失败，请保持设备静止并再次点击右上角磁力计图标"
        }
    }

    private fun formatMagneticAccuracy(accuracy: Int): String = when (accuracy) {
        SensorManager.SENSOR_STATUS_ACCURACY_HIGH -> "高(3)"
        SensorManager.SENSOR_STATUS_ACCURACY_MEDIUM -> "中(2)"
        SensorManager.SENSOR_STATUS_ACCURACY_LOW -> "低(1)"
        SensorManager.SENSOR_STATUS_UNRELIABLE -> "未标定(0)"
        else -> "未知($accuracy)"
    }

    private fun formatMagneticDispersion(dispersionDegrees: Float): String {
        return if (dispersionDegrees.isFinite()) {
            String.format(Locale.US, "%.1f°", dispersionDegrees)
        } else {
            "--"
        }
    }

    private fun normalizeRadians(value: Double): Double {
        val fullTurn = Math.PI * 2.0
        var normalized = value % fullTurn
        if (normalized < 0.0) normalized += fullTurn
        return normalized
    }

    private fun clearMapMeasurement() {
        measurementGeneration++
        measurementTextures.clear()
        currentMapMeasurement = null
        measurementCalculationDirty = isMeasurementCalculationRunning
        mapView.setMapMeasurement(null)
        tvMapMeasurement.text = "地图尺寸：--"
    }

    private fun replaceMeasurementTextures(textures: List<SubmapTexture>) {
        measurementTextures.clear()
        for (texture in textures) measurementTextures[texture.key] = texture
        measurementGeneration++
        requestMapMeasurementCalculation()
    }

    private fun mergeMeasurementTextures(textures: List<SubmapTexture>) {
        var changed = false
        for (texture in textures) {
            val previous = measurementTextures[texture.key]
            if (previous == null || previous.version != texture.version ||
                !previous.hasSameMeasurementPose(texture)) {
                measurementTextures[texture.key] = texture
                changed = true
            }
        }
        if (!changed) return
        measurementGeneration++
        requestMapMeasurementCalculation()
    }

    private fun requestMapMeasurementCalculation() {
        if (isMeasurementCalculationRunning) {
            measurementCalculationDirty = true
            return
        }
        val snapshot = measurementTextures.values.toList()
        if (snapshot.isEmpty()) {
            currentMapMeasurement = null
            mapView.setMapMeasurement(null)
            tvMapMeasurement.text = "地图尺寸：--"
            return
        }

        val generation = measurementGeneration
        isMeasurementCalculationRunning = true
        measurementCalculationDirty = false
        if (currentMapMeasurement == null) tvMapMeasurement.text = "正在计算地图尺寸…"
        Thread {
            val measurement = try {
                MapMeasurementCalculator.calculate(snapshot)
            } catch (e: Exception) {
                Log.e("MapMeasurement", "计算地图尺寸失败", e)
                null
            }
            runOnUiThread {
                isMeasurementCalculationRunning = false
                if (generation == measurementGeneration) {
                    currentMapMeasurement = measurement
                    mapView.setMapMeasurement(measurement)
                    updateMeasurementDisplay()
                }
                if (measurementCalculationDirty || generation != measurementGeneration) {
                    measurementCalculationDirty = false
                    requestMapMeasurementCalculation()
                }
            }
        }.start()
    }

    private fun formatMapMeasurement(measurement: MapMeasurement?): String {
        if (measurement == null) return "地图尺寸：--"
        if (measurement.hasSharedFloorPlanGeometry()) {
            return String.format(
                Locale.US,
                "地图/户型图统一边界：长 %.2f m　宽 %.2f m\n闭合轮廓面积：%.2f m²",
                measurement.lengthMeters,
                measurement.widthMeters,
                measurement.boundingAreaSquareMeters
            )
        }
        return String.format(
            Locale.US,
            "地图全局范围：%.2f m × %.2f m\n长：%.2f m　宽：%.2f m\n矩形范围面积：%.2f m²",
            measurement.xSizeMeters,
            measurement.ySizeMeters,
            measurement.lengthMeters,
            measurement.widthMeters,
            measurement.boundingAreaSquareMeters
        )
    }

    private fun SubmapTexture.hasSameMeasurementPose(other: SubmapTexture): Boolean {
        return kotlin.math.abs(originX - other.originX) < 1e-4f &&
            kotlin.math.abs(originY - other.originY) < 1e-4f &&
            kotlin.math.abs(theta - other.theta) < 1e-4f
    }

    private fun generateFloorPlan(
        timestamp: Long,
        export: FloorPlanExportResult
    ): FloorPlanGenerationResult? {
        val output = File(export.sessionDir, "floorplan_result_$timestamp.png")
        val workDir = File(export.sessionDir, "work")
        val generation = floorPlanNative.generate(
            input = export.input,
            visualInput = export.visual,
            semanticInput = export.semantic,
            output = output,
            workDir = workDir,
            metersPerPixel = export.geometry.resolutionMetersPerPixel,
            trajectoryPixels = export.trajectoryPixels
        )
        if (generation == null) {
            return generateFallbackFloorPlan(output, export)
        }
        if (!output.exists()) return null
        var annotation = FloorPlanImageAnnotator.annotateFile(
            file = output,
            generation = generation,
            metersPerPixel = export.geometry.resolutionMetersPerPixel
        )
        if (!annotation.annotated) {
            Log.w("FloorPlan", "户型图尺寸标注失败：${annotation.failureReason}")
            annotation = FloorPlanImageAnnotator.annotateFallbackFile(
                file = output,
                metersPerPixel = export.geometry.resolutionMetersPerPixel
            )
        }
        val length = annotation.lengthMeters.takeIf { it.isFinite() && it > 0f }
            ?: (generation.dimensionLongSizePixels *
                export.geometry.resolutionMetersPerPixel)
        val width = annotation.widthMeters.takeIf { it.isFinite() && it > 0f }
            ?: (generation.dimensionShortSizePixels *
                export.geometry.resolutionMetersPerPixel)
        val footprintArea = generation.footprintAreaPixelsSquared *
            export.geometry.resolutionMetersPerPixel *
            export.geometry.resolutionMetersPerPixel
        val dimensions = FloorPlanDimensions(
            lengthMeters = length,
            widthMeters = width,
            areaSquareMeters = footprintArea,
            source = FloorPlanDimensionSource.FITTED_OUTLINE
        ).takeIf { it.isValid() }
        val sharedMapMeasurement = MapMeasurementCalculator.fromFloorPlan(
            generation = generation,
            geometry = export.geometry
        )
        val clippedOccupancy = File(workDir, "occupancy_clipped.png")
            .takeIf { it.exists() }
            ?.let { BitmapFactory.decodeFile(it.absolutePath) }
        if (clippedOccupancy != null) {
            runOnUiThread {
                mapView.setFinalizedOccupancyBitmap(clippedOccupancy)
            }
        }

        val displayFloorPlanOverlayFile = File(workDir, "floorplan_display_overlay.png")
        File(workDir, "floorplan_overlay.png")
            .takeIf { it.exists() }
            ?.let { BitmapFactory.decodeFile(it.absolutePath) }
            ?.let { sourceOverlay ->
                val labeled = FloorPlanDimensionOverlayRenderer.render(
                    floorPlanOverlay = sourceOverlay,
                    generation = generation,
                    metersPerPixel = export.geometry.resolutionMetersPerPixel
                )
                sourceOverlay.recycle()
                if (labeled != null) {
                    try {
                        FileOutputStream(displayFloorPlanOverlayFile).use { stream ->
                            if (!labeled.compress(Bitmap.CompressFormat.PNG, 100, stream)) {
                                throw IOException("透明户型图层写入失败")
                            }
                        }
                    } finally {
                        labeled.recycle()
                    }
                }
            }

        val heatMapOverlayFile = File(workDir, "heatmap_overlay.png")
        HeatMapRenderer.render(
            samples = rssiSamples.toList(),
            geometry = export.geometry,
            outlinePixels = generation.outlineVerticesPixels
        )?.let { heatMap ->
            try {
                FileOutputStream(heatMapOverlayFile).use { stream ->
                    if (!heatMap.compress(Bitmap.CompressFormat.PNG, 100, stream)) {
                        throw IOException("透明热力图层写入失败")
                    }
                }
            } finally {
                heatMap.recycle()
            }
        } ?: Log.i("HeatMap", "有效 RSSI 样本不足，跳过热力图层")

        val trajectoryOverlayFile = File(workDir, "trajectory_overlay.png")
        TrajectoryOverlayRenderer.render(
            width = export.geometry.widthPx,
            height = export.geometry.heightPx,
            points = export.trajectoryPixels
        )?.let { trajectoryOverlay ->
            try {
                FileOutputStream(trajectoryOverlayFile).use { stream ->
                    if (!trajectoryOverlay.compress(Bitmap.CompressFormat.PNG, 100, stream)) {
                        throw IOException("运动轨迹图层写入失败")
                    }
                }
            } finally {
                trajectoryOverlay.recycle()
            }
        }
        return FloorPlanGenerationResult(
            file = output,
            annotation = annotation,
            dimensions = dimensions,
            outlineClosed = generation.outlineClosed,
            sharedMapMeasurement = sharedMapMeasurement,
            structuralMap = File(workDir, "best_structural_map.png")
                .takeIf { it.exists() },
            floorPlanOverlay = displayFloorPlanOverlayFile.takeIf { it.exists() },
            heatMapOverlay = heatMapOverlayFile.takeIf { it.exists() },
            trajectoryOverlay = trajectoryOverlayFile.takeIf { it.exists() }
        )
    }

    private fun generateFallbackFloorPlan(
        output: File,
        export: FloorPlanExportResult
    ): FloorPlanGenerationResult? {
        return try {
            export.visual.copyTo(output, overwrite = true)
            val annotation = FloorPlanImageAnnotator.annotateFallbackFile(
                file = output,
                metersPerPixel = export.geometry.resolutionMetersPerPixel
            )
            val fallbackDimensions = if (
                annotation.lengthMeters.isFinite() && annotation.lengthMeters > 0f &&
                annotation.widthMeters.isFinite() && annotation.widthMeters > 0f &&
                annotation.boundingAreaSquareMeters.isFinite() &&
                annotation.boundingAreaSquareMeters > 0f
            ) {
                FloorPlanDimensions(
                    lengthMeters = annotation.lengthMeters,
                    widthMeters = annotation.widthMeters,
                    areaSquareMeters = annotation.boundingAreaSquareMeters,
                    source = FloorPlanDimensionSource.MAP_PREVIEW
                )
            } else {
                export.previewDimensions
            }
            FloorPlanGenerationResult(
                file = output,
                annotation = annotation,
                dimensions = fallbackDimensions,
                outlineClosed = annotation.annotated,
                sharedMapMeasurement = null,
                structuralMap = null
            )
        } catch (e: Exception) {
            Log.e("FloorPlan", "生成户型图最终尺寸兜底失败", e)
            null
        }
    }

    private fun updateCurrentFloorPlan(
        export: FloorPlanExportResult?,
        generated: FloorPlanGenerationResult?
    ): String {
        val selected = generated?.file?.takeIf { it.exists() }
            // The visual fallback is the complete occupancy-grid presentation:
            // gray unknown, white explored floor and black occupied boundary.
            // Algorithm input remains a separate wall-only structural raster.
            ?: export?.visual?.takeIf { it.exists() }
        if (selected == null) {
            currentFloorPlanResult = null
            currentFloorPlanDimensions = null
            currentFloorPlanWarning = null
            btnSaveFloorPlan.isEnabled = false
            clearFloorPlanLayers()
            updateMeasurementDisplay()
            return "户型图 PNG 导出失败"
        }

        val layerExport = export ?: run {
            clearFloorPlanLayers()
            currentFloorPlanResult = null
            btnSaveFloorPlan.isEnabled = false
            return "点云图层数据缺失"
        }
        val pointCloud = BitmapFactory.decodeFile(layerExport.visual.absolutePath)
        if (pointCloud == null) {
            clearFloorPlanLayers()
            currentFloorPlanResult = null
            btnSaveFloorPlan.isEnabled = false
            return "点云图层解码失败"
        }

        currentFloorPlanResult = selected
        currentFloorPlanDimensions = generated?.dimensions ?: layerExport.previewDimensions
        generated?.sharedMapMeasurement?.let { sharedMeasurement ->
            // Invalidate any older asynchronous raster-bounds calculation and
            // drive the map overlay from the same fitted outline used by the
            // floor-plan PNG.
            measurementGeneration++
            currentMapMeasurement = sharedMeasurement
            mapView.setMapMeasurement(sharedMeasurement)
        }
        currentFloorPlanWarning = when {
            generated == null -> "绿色拟合户型图生成失败，已回退到完整占据栅格图"
            !generated.annotation.annotated -> "户型图已生成，但尺寸标注失败：" +
                (generated.annotation.failureReason ?: "未知原因")
            !generated.outlineClosed -> "拟合轮廓未完全闭合，尺寸按绿色拟合边界外接范围计算"
            else -> null
        }
        btnSaveFloorPlan.isEnabled = true
        setFloorPlanLayers(
            FloorPlanLayers(
                width = layerExport.geometry.widthPx,
                height = layerExport.geometry.heightPx,
                pointCloud = pointCloud,
                heatMap = generated?.heatMapOverlay?.let {
                    BitmapFactory.decodeFile(it.absolutePath)
                },
                trajectory = generated?.trajectoryOverlay?.let {
                    BitmapFactory.decodeFile(it.absolutePath)
                },
                floorPlan = generated?.floorPlanOverlay?.let {
                    BitmapFactory.decodeFile(it.absolutePath)
                }
            )
        )
        updateMeasurementDisplay()

        val resultDescription = when {
            generated?.annotation?.annotated == true -> "原效果户型图及四边尺寸标注已生成"
            generated != null -> "原效果户型图已生成，可使用原按钮保存"
            else -> "绿色拟合结果不可用，完整占据栅格图仍可使用原按钮保存"
        }
        val dimensionsText = formatFloorPlanDimensions(currentFloorPlanDimensions)
        val warningText = currentFloorPlanWarning?.let { "\n$it" }.orEmpty()
        return "$resultDescription\n$dimensionsText$warningText"
    }

    private fun setFloorPlanLayers(layers: FloorPlanLayers) {
        clearFloorPlanLayers()
        floorPlanLayers = layers
        checkPointCloud.isEnabled = layers.pointCloud != null
        checkFloorPlanOverlay.isEnabled = layers.floorPlan != null
        checkHeatMapOverlay.isEnabled = layers.heatMap != null
        checkTrajectoryOverlay.isEnabled = layers.trajectory != null
        checkHeatMapOverlay.text = if (layers.heatMap != null) {
            "热力图"
        } else {
            "热力图（无数据）"
        }
        refreshFloorPlanLayerPreview()
    }

    private fun refreshFloorPlanLayerPreview() {
        val layers = floorPlanLayers ?: return
        val output = FloorPlanLayerComposer.compose(
            layers,
            FloorPlanLayerVisibility(
                pointCloud = checkPointCloud.isChecked,
                heatMap = checkHeatMapOverlay.isChecked,
                trajectory = checkTrajectoryOverlay.isChecked,
                floorPlan = checkFloorPlanOverlay.isChecked
            )
        ) ?: return
        val previous = floorPlanDisplayBitmap
        floorPlanDisplayBitmap = output
        floorPlanView.setImageBitmap(output)
        if (previous !== output && previous?.isRecycled == false) previous.recycle()
    }

    private fun clearFloorPlanLayers() {
        floorPlanView.setImageBitmap(null)
        floorPlanDisplayBitmap?.takeIf { !it.isRecycled }?.recycle()
        floorPlanDisplayBitmap = null
        floorPlanLayers?.let { layers ->
            listOfNotNull(
                layers.pointCloud,
                layers.heatMap,
                layers.trajectory,
                layers.floorPlan
            ).distinct().forEach { bitmap ->
                if (!bitmap.isRecycled) bitmap.recycle()
            }
        }
        floorPlanLayers = null
        if (::checkHeatMapOverlay.isInitialized) {
            checkHeatMapOverlay.text = "热力图"
        }
    }

    private fun formatFloorPlanDimensions(dimensions: FloorPlanDimensions?): String {
        if (dimensions == null || !dimensions.isValid()) return "户型图尺寸/面积：--"
        val label = if (dimensions.source == FloorPlanDimensionSource.FITTED_OUTLINE) {
            "户型图绿色拟合边界"
        } else {
            "户型图预览范围"
        }
        val areaLabel = if (dimensions.source == FloorPlanDimensionSource.FITTED_OUTLINE) {
            "闭合轮廓面积"
        } else {
            "范围面积"
        }
        return String.format(
            Locale.US,
            "%s：长 %.2f m × 宽 %.2f m，%s %.2f m²",
            label,
            dimensions.lengthMeters,
            dimensions.widthMeters,
            areaLabel,
            dimensions.areaSquareMeters
        )
    }

    private fun updateMeasurementDisplay() {
        val mapText = formatMapMeasurement(currentMapMeasurement)
        val floorPlanText = currentFloorPlanDimensions?.let { "\n${formatFloorPlanDimensions(it)}" }
            .orEmpty()
        tvMapMeasurement.text = mapText + floorPlanText
    }

    private fun saveCurrentFloorPlan() {
        val source = currentFloorPlanResult
        if (source == null || !source.exists()) {
            Toast.makeText(this, "还没有可保存的户型图", Toast.LENGTH_SHORT).show()
            return
        }

        if (Build.VERSION.SDK_INT <= Build.VERSION_CODES.P &&
            ContextCompat.checkSelfPermission(this, Manifest.permission.WRITE_EXTERNAL_STORAGE) !=
            PackageManager.PERMISSION_GRANTED) {
            pendingLegacyFloorPlanSave = source
            legacyStoragePermissionLauncher.launch(Manifest.permission.WRITE_EXTERNAL_STORAGE)
            return
        }
        saveFloorPlanFile(source)
    }

    private fun saveFloorPlanFile(source: File) {
        if (!source.exists()) {
            show("还没有可保存的户型图")
            return
        }
        val dimensions = currentFloorPlanDimensions
        val warning = currentFloorPlanWarning
        btnSaveFloorPlan.isEnabled = false
        Thread {
            try {
                val location = writeFloorPlanToDownloads(source)
                runOnUiThread {
                    btnSaveFloorPlan.isEnabled = currentFloorPlanResult?.exists() == true
                    updateMeasurementDisplay()
                    val dimensionsText = formatFloorPlanDimensions(dimensions)
                    val warningText = warning?.let { "\n注意：$it" }.orEmpty()
                    show("户型图已保存：$location\n$dimensionsText$warningText")
                }
            } catch (e: Exception) {
                Log.e("CartographerJNI", "保存户型图失败", e)
                runOnUiThread {
                    btnSaveFloorPlan.isEnabled = currentFloorPlanResult?.exists() == true
                    show("保存户型图失败：${e.message ?: "未知错误"}")
                }
            }
        }.start()
    }

    private fun writeFloorPlanToDownloads(source: File): String {
        val displayName = "floorplan_saved_${System.currentTimeMillis()}.png"
        return writeFileToDownloads(source, displayName, "image/png", "户型图")
    }

    private fun writeMapToDownloads(source: File): String =
        writeFileToDownloads(
            source = source,
            displayName = source.name,
            mimeType = "application/octet-stream",
            description = "地图"
        )

    private fun writeFileToDownloads(
        source: File,
        displayName: String,
        mimeType: String,
        description: String
    ): String {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
            val values = ContentValues().apply {
                put(MediaStore.MediaColumns.DISPLAY_NAME, displayName)
                put(MediaStore.MediaColumns.MIME_TYPE, mimeType)
                put(
                    MediaStore.MediaColumns.RELATIVE_PATH,
                    "${Environment.DIRECTORY_DOWNLOADS}/CartographerMaps"
                )
                put(MediaStore.MediaColumns.IS_PENDING, 1)
            }
            val resolver = contentResolver
            val uri = resolver.insert(MediaStore.Downloads.EXTERNAL_CONTENT_URI, values)
                ?: throw IOException("无法在 Downloads 中创建$description")
            try {
                resolver.openOutputStream(uri, "w")?.use { output ->
                    source.inputStream().use { input -> input.copyTo(output) }
                } ?: throw IOException("无法打开$description 保存位置")
                values.clear()
                values.put(MediaStore.MediaColumns.IS_PENDING, 0)
                resolver.update(uri, values, null, null)
            } catch (e: Exception) {
                resolver.delete(uri, null, null)
                throw e
            }
            return "${Environment.DIRECTORY_DOWNLOADS}/CartographerMaps/$displayName"
        }

        val publicDir = File(
            Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS),
            "CartographerMaps"
        )
        if (!publicDir.exists() && !publicDir.mkdirs()) {
            throw IOException("无法创建 Downloads/CartographerMaps")
        }
        val output = File(publicDir, displayName)
        source.copyTo(output, overwrite = false)
        MediaScannerConnection.scanFile(
            this,
            arrayOf(output.absolutePath),
            arrayOf(mimeType),
            null
        )
        return output.absolutePath
    }

    private fun exportFloorPlanMaps(
        timestamp: Long,
        textures: List<SubmapTexture>
    ): FloorPlanExportResult? {
        if (textures.isEmpty()) return null
        // Render the finalized submaps once. The on-screen final map and all
        // floor-plan inputs now use the same confidence fusion, dimensions and
        // world-to-pixel transform instead of independently compositing raw
        // submaps with four different alpha decisions.
        val fusedRender = FusedMapRenderer.render(textures, finalized = true)
            ?: return null
        val renders = try {
            val input = FloorPlanMapExporter.renderFusedWithGeometry(
                fusedRender,
                FloorPlanMapExporter.Style.ALGORITHM_INPUT
            ) ?: return null
            val visual = FloorPlanMapExporter.renderFusedWithGeometry(
                fusedRender,
                FloorPlanMapExporter.Style.VISUAL_MAP
            ) ?: run {
                input.bitmap.recycle()
                return null
            }
            val semantic = FloorPlanMapExporter.renderFusedWithGeometry(
                fusedRender,
                FloorPlanMapExporter.Style.SEMANTIC
            ) ?: run {
                input.bitmap.recycle()
                visual.bitmap.recycle()
                return null
            }
            val preview = FloorPlanMapExporter.renderFusedWithGeometry(
                fusedRender,
                FloorPlanMapExporter.Style.PREVIEW
            ) ?: run {
                input.bitmap.recycle()
                visual.bitmap.recycle()
                semantic.bitmap.recycle()
                return null
            }
            arrayOf(input, visual, semantic, preview)
        } finally {
            fusedRender.bitmap.recycle()
        }
        val inputRender = renders[0]
        val visualRender = renders[1]
        val semanticRender = renders[2]
        val previewBitmap = renders[3].bitmap
        val sessionDir = File(filesDir, "floorplans/$timestamp")
        if (!sessionDir.exists() && !sessionDir.mkdirs()) {
            inputRender.bitmap.recycle()
            visualRender.bitmap.recycle()
            semanticRender.bitmap.recycle()
            previewBitmap.recycle()
            throw IOException("无法创建户型图内部目录")
        }
        val inputOutput = File(sessionDir, "floorplan_input_$timestamp.png")
        val visualOutput = File(sessionDir, "floorplan_visual_$timestamp.png")
        val semanticOutput = File(sessionDir, "floorplan_semantic_$timestamp.png")
        val previewOutput = File(sessionDir, "floorplan_preview_$timestamp.png")
        try {
            FileOutputStream(inputOutput).use { stream ->
                if (!inputRender.bitmap.compress(Bitmap.CompressFormat.PNG, 100, stream)) {
                    throw IOException("户型图算法输入写入失败")
                }
            }
            FileOutputStream(visualOutput).use { stream ->
                if (!visualRender.bitmap.compress(Bitmap.CompressFormat.PNG, 100, stream)) {
                    throw IOException("户型图视觉底图写入失败")
                }
            }
            FileOutputStream(semanticOutput).use { stream ->
                if (!semanticRender.bitmap.compress(Bitmap.CompressFormat.PNG, 100, stream)) {
                    throw IOException("户型图语义图写入失败")
                }
            }
            FileOutputStream(previewOutput).use { stream ->
                if (!previewBitmap.compress(Bitmap.CompressFormat.PNG, 100, stream)) {
                    throw IOException("户型图预览写入失败")
                }
            }
        } finally {
            inputRender.bitmap.recycle()
            visualRender.bitmap.recycle()
            semanticRender.bitmap.recycle()
            previewBitmap.recycle()
        }
        val trajectoryPixels = carto.getTrajectoryNodePoses().mapNotNull { point ->
            val pixelX = inputRender.geometry.worldToPixelX(point.x)
            val pixelY = inputRender.geometry.worldToPixelY(point.y)
            if (pixelX.isFinite() && pixelY.isFinite() &&
                pixelX >= 0f && pixelY >= 0f &&
                pixelX < inputRender.geometry.widthPx &&
                pixelY < inputRender.geometry.heightPx) {
                FloorPlanPixelPoint(pixelX, pixelY)
            } else {
                null
            }
        }
        val previewDimensions = try {
            MapMeasurementCalculator.calculate(textures)?.let {
                FloorPlanDimensions(
                    lengthMeters = it.lengthMeters,
                    widthMeters = it.widthMeters,
                    areaSquareMeters = it.boundingAreaSquareMeters,
                    source = FloorPlanDimensionSource.MAP_PREVIEW
                )
            }
        } catch (e: Exception) {
            Log.w("FloorPlan", "计算户型图预览范围失败", e)
            null
        }
        return FloorPlanExportResult(
            input = inputOutput,
            visual = visualOutput,
            semantic = semanticOutput,
            preview = previewOutput,
            geometry = inputRender.geometry,
            sessionDir = sessionDir,
            trajectoryPixels = trajectoryPixels,
            previewDimensions = previewDimensions,
            textures = textures
        )
    }

    private data class FloorPlanExportResult(
        val input: File,
        val visual: File,
        val semantic: File,
        val preview: File,
        val geometry: FloorPlanMapExporter.ExportGeometry,
        val sessionDir: File,
        val trajectoryPixels: List<FloorPlanPixelPoint>,
        val previewDimensions: FloorPlanDimensions?,
        val textures: List<SubmapTexture>
    )

    private data class FloorPlanGenerationResult(
        val file: File,
        val annotation: FloorPlanImageAnnotator.Result,
        val dimensions: FloorPlanDimensions?,
        val outlineClosed: Boolean,
        val sharedMapMeasurement: MapMeasurement?,
        val structuralMap: File?,
        val floorPlanOverlay: File? = null,
        val heatMapOverlay: File? = null,
        val trajectoryOverlay: File? = null
    )

    private enum class FloorPlanDimensionSource {
        FITTED_OUTLINE,
        MAP_PREVIEW
    }

    private data class FloorPlanDimensions(
        val lengthMeters: Float,
        val widthMeters: Float,
        val areaSquareMeters: Float,
        val source: FloorPlanDimensionSource
    ) {
        fun isValid(): Boolean = lengthMeters.isFinite() && widthMeters.isFinite() &&
            areaSquareMeters.isFinite() && lengthMeters > 0f && widthMeters > 0f &&
            areaSquareMeters > 0f
    }

    private val usbReceiver = object : BroadcastReceiver() {
        override fun onReceive(c: Context, i: Intent) {
            if (i.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)) {
                val dev = i.getParcelableExtra<UsbDevice>(UsbManager.EXTRA_DEVICE)
                dev?.let {
                    val driver = UsbSerialProber.getDefaultProber().probeDevice(it)
                    openPort(driver!!.ports[0])
                }
            }
        }
    }

    private fun sendLidarCommand(port: UsbSerialPort, cmd1: Int, cmd2: Int) {
        val command = byteArrayOf(0xAA.toByte(), 0x55.toByte(), cmd1.toByte(), cmd2.toByte())
        port.write(command, 100)
        Log.i("LidarFinal", "发送雷达命令：AA 55 %02X %02X".format(cmd1, cmd2))
    }

    override fun onAccuracyChanged(s: Sensor?, a: Int) {}
    override fun onDestroy() {
        clearFloorPlanLayers()
        super.onDestroy()
        isMagneticCalibrationRunning = false
        magneticHeadingProvider.stop()
        sensorManager.unregisterListener(this)
        usbIoManager?.stop()
        usbSerialPort?.let {
            try {
                sendLidarCommand(it, 0xF5, 0x0A)
            } catch (_: Exception) {
            }
        }
        usbSerialPort?.close()
        unregisterReceiver(usbReceiver)
        carto.reset()
    }

    private fun show(msg: String) {
        runOnUiThread {
            Toast.makeText(this, msg, Toast.LENGTH_SHORT).show()
        }
    }
}
