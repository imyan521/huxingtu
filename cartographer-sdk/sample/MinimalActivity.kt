package customer.example

import android.os.Bundle
import android.util.Log
import android.view.View
import android.widget.Button
import android.widget.LinearLayout
import android.widget.ScrollView
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import com.cartographer.sdk.*
import java.io.File
import java.util.Locale

/**
 * Complete custom-UI example for the public Cartographer SDK API.
 *
 * This Activity deliberately uses only classes from `com.cartographer.sdk`.
 * Production code can replace this UI while keeping the lifecycle ordering.
 */
class MinimalActivity : AppCompatActivity() {
    private lateinit var sdk: CartographerSdk
    private lateinit var statusView: TextView
    private lateinit var requestButton: Button
    private lateinit var startButton: Button
    private lateinit var stopButton: Button
    private lateinit var outputActions: LinearLayout

    private var sdkReady = false
    private var lidarConnected = false
    private var mapping = false
    private var scanCount = 0L

    private val mapFile by lazy { File(filesDir, "maps/current.pbstream") }
    private val finalMapFile by lazy { File(filesDir, "exports/final_map.png") }
    private val floorPlanFile by lazy { File(filesDir, "exports/floor_plan.png") }
    private val floorPlanWorkDir by lazy { File(cacheDir, "floor_plan") }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(createContentView())
        sdk = CartographerSdk.initialize(
            applicationContext,
            SdkConfig(snapshotIntervalMs = 100L, imuMinimumIntervalNs = 5_000_000L),
            createListener()
        )
        appendStatus("SDK ${CartographerSdk.SDK_VERSION} 正在初始化……")
        updateButtons()
    }

    private fun createListener() = object : CartographerListener {
        override fun onUsbDeviceFound(deviceName: String) {
            appendStatus("发现雷达：$deviceName")
        }

        override fun onUsbPermissionResult(granted: Boolean) {
            appendStatus(if (granted) "USB 权限已授予" else "USB 权限被拒绝")
            if (granted) checkResult("连接雷达", sdk.connect())
        }

        override fun onConnectionStateChanged(state: ConnectionState) {
            lidarConnected = state == ConnectionState.CONNECTED
            appendStatus("连接状态：$state")
            updateButtons()
        }

        override fun onMappingStateChanged(state: MappingState) {
            sdkReady = true
            mapping = state == MappingState.MAPPING
            appendStatus("建图状态：$state")
            updateButtons()
        }

        override fun onLidarScan(scan: LidarScan) {
            scanCount++
            // Avoid expensive UI work on every scan in production.
            if (scanCount % 10L == 0L) {
                appendStatus("雷达帧：$scanCount，当前点数：${scan.rangesMeters.size}")
            }
        }

        override fun onSnapshot(snapshot: SdkSnapshot) {
            val pose = snapshot.pose ?: return
            Log.d(
                TAG,
                "pose=(%.2f, %.2f, %.1f°), frames=%d, imu=%d, nodes=%d".format(
                    Locale.US, pose.x, pose.y, Math.toDegrees(pose.theta),
                    snapshot.rangeFrames, snapshot.imuSamples, snapshot.insertedNodes
                )
            )
        }

        override fun onError(code: SdkError, message: String, cause: Throwable?) {
            Log.e(TAG, "$code: $message", cause)
            appendStatus("错误 $code：$message")
        }
    }

    private fun requestUsbAndConnect() {
        checkResult("申请 USB 权限", sdk.requestUsbPermission(this))
    }

    private fun startMapping() {
        when {
            !sdkReady -> appendStatus("SDK 尚未 READY，请稍候")
            !lidarConnected -> appendStatus("请先连接雷达")
            else -> checkResult("开始建图", sdk.startMapping())
        }
    }

    private fun stopMapping() {
        // Wait for MappingState.FINISHED before saving/exporting. stopMapping()
        // finishes the trajectory and final optimization asynchronously.
        checkResult("结束建图", sdk.stopMapping())
    }

    private fun saveMap() {
        sdk.saveMap(mapFile) { reportAsyncResult("保存地图", it, mapFile) }
    }

    private fun loadMap() {
        if (!mapFile.isFile) {
            appendStatus("没有可加载的地图：${mapFile.absolutePath}")
            return
        }
        sdk.loadMap(mapFile) { reportAsyncResult("加载地图", it, mapFile) }
    }

    private fun inspectMapData() {
        checkResult("读取活动子图", sdk.getActiveSubmapTextures {
            appendStatus("活动子图：${it.size}")
        })
        checkResult("读取最新子图", sdk.getLatestSubmapTexture { texture ->
            appendStatus(texture?.let {
                "最新子图：${it.trajectoryId}:${it.submapIndex}，${it.width}×${it.height}"
            } ?: "暂无最新子图")
        })
        checkResult("读取全部子图", sdk.getSubmapTextures {
            appendStatus("全部子图：${it.size}")
        })
        checkResult("读取轨迹", sdk.getTrajectory {
            appendStatus("轨迹节点：${it.size}")
        })
        checkResult("读取重定位状态", sdk.getRelocalizationStatus {
            appendStatus(
                "重定位=${it.isRelocalized}，活动节点=${it.activeNodeCount}，" +
                    "跨轨迹约束=${it.interTrajectoryConstraintCount}"
            )
        })
    }

    private fun exportFinalRaster() {
        checkResult("导出最终栅格图", sdk.getFinalRasterMap(finalMapFile) { result, map ->
            if (result.isSuccess && map != null) {
                appendStatus(
                    "最终栅格图：${map.outputFile?.absolutePath}，" +
                        "${map.geometry.widthPixels}×${map.geometry.heightPixels}"
                )
                // FinalRasterMap.bitmap is owned by the caller.
                map.bitmap.recycle()
            } else appendStatus("最终栅格图失败：${result.error} ${result.message.orEmpty()}")
        })
    }

    private fun generateFloorPlan() {
        checkResult(
            "生成户型图",
            sdk.generateFloorPlan(floorPlanFile, floorPlanWorkDir) { result, plan ->
                if (result.isSuccess && plan != null) {
                    val size = plan.dimensions
                    appendStatus(
                        "户型图：${plan.imageFile.absolutePath}，" +
                            "%.2f×%.2fm，面积 %.2fm²，闭合=${plan.outlineClosed}".format(
                                Locale.US, size.lengthMeters, size.widthMeters,
                                size.areaSquareMeters
                            )
                    )
                } else appendStatus("户型图失败：${result.error} ${result.message.orEmpty()}")
            }
        )
    }

    private fun checkResult(action: String, result: SdkResult) {
        if (result.isSuccess) appendStatus("$action：请求已接受")
        else appendStatus("$action 失败：${result.error} ${result.message.orEmpty()}")
    }

    private fun reportAsyncResult(action: String, result: SdkResult, file: File) {
        if (result.isSuccess) appendStatus("$action 成功：${file.absolutePath}")
        else appendStatus("$action 失败：${result.error} ${result.message.orEmpty()}")
    }

    private fun createContentView(): View {
        val content = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(32, 32, 32, 32)
        }
        statusView = TextView(this).apply { text = "Cartographer SDK 示例\n" }
        requestButton = content.addAction("1. USB授权并连接", ::requestUsbAndConnect)
        startButton = content.addAction("2. 开始建图", ::startMapping)
        stopButton = content.addAction("3. 结束并优化", ::stopMapping)
        outputActions = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL }
        outputActions.addAction("4. 保存 PBStream", ::saveMap)
        outputActions.addAction("5. 加载 PBStream（继续建图前）", ::loadMap)
        outputActions.addAction("6. 查询子图/轨迹/重定位", ::inspectMapData)
        outputActions.addAction("7. 导出最终栅格图", ::exportFinalRaster)
        outputActions.addAction("8. 生成户型图", ::generateFloorPlan)
        content.addView(outputActions)
        content.addAction("断开雷达") { checkResult("断开雷达", sdk.disconnect()) }
        content.addView(statusView)
        return ScrollView(this).apply { addView(content) }
    }

    private fun LinearLayout.addAction(label: String, action: () -> Unit): Button =
        Button(this@MinimalActivity).also {
            it.text = label
            it.setOnClickListener { action() }
            addView(it)
        }

    private fun updateButtons() {
        if (!::requestButton.isInitialized) return
        requestButton.isEnabled = !lidarConnected && !mapping
        startButton.isEnabled = sdkReady && lidarConnected && !mapping
        stopButton.isEnabled = mapping
        outputActions.isEnabled = sdkReady && !mapping
        for (index in 0 until outputActions.childCount) {
            outputActions.getChildAt(index).isEnabled = outputActions.isEnabled
        }
    }

    private fun appendStatus(message: String) {
        Log.i(TAG, message)
        if (::statusView.isInitialized) statusView.append("$message\n")
    }

    override fun onDestroy() {
        if (::sdk.isInitialized) sdk.close()
        super.onDestroy()
    }

    companion object {
        private const val TAG = "CartographerSdkSample"
    }
}
