package com.cartographer.demo

import java.io.File

data class FloorPlanPixelPoint(val x: Float, val y: Float)

data class FloorPlanGenerationInfo(
    val outlineClosed: Boolean,
    val outlineWidthPixels: Float,
    val outlineHeightPixels: Float,
    val outlineLeftPixels: Float,
    val outlineTopPixels: Float,
    val outlineRightPixels: Float,
    val outlineBottomPixels: Float,
    val rotationDegrees: Float,
    val supportRatio: Float,
    val vertexCount: Int,
    val closeSizePixels: Int,
    val dimensionCenterXPixels: Float,
    val dimensionCenterYPixels: Float,
    val dimensionLongAxisX: Float,
    val dimensionLongAxisY: Float,
    val dimensionShortAxisX: Float,
    val dimensionShortAxisY: Float,
    val dimensionLongSizePixels: Float,
    val dimensionShortSizePixels: Float,
    val outlineVerticesPixels: List<FloorPlanPixelPoint>,
    val footprintAreaPixelsSquared: Float,
    val footprintPerimeterPixels: Float
)

class FloorPlanNative {

    init {
        System.loadLibrary("opencv_java4")
        System.loadLibrary("floorplan-jni")
    }

    fun generate(
        input: File,
        visualInput: File,
        semanticInput: File,
        output: File,
        workDir: File,
        metersPerPixel: Float,
        trajectoryPixels: List<FloorPlanPixelPoint>
    ): FloorPlanGenerationInfo? {
        if (!input.exists() || !visualInput.exists() || !semanticInput.exists() ||
            !metersPerPixel.isFinite() || metersPerPixel <= 0f ||
            trajectoryPixels.isEmpty()) return null
        if (!workDir.exists()) workDir.mkdirs()
        output.parentFile?.mkdirs()
        val trajectoryValues = DoubleArray(trajectoryPixels.size * 2)
        trajectoryPixels.forEachIndexed { index, point ->
            trajectoryValues[index * 2] = point.x.toDouble()
            trajectoryValues[index * 2 + 1] = point.y.toDouble()
        }
        val values = nativeGenerateFloorPlan(
            input.absolutePath,
            visualInput.absolutePath,
            semanticInput.absolutePath,
            output.absolutePath,
            workDir.absolutePath,
            metersPerPixel.toDouble(),
            trajectoryValues
        )
        if (values == null || values.size < 19 || values.any { !it.isFinite() }) return null
        val vertexCount = values[9].toInt()
        val polygonEnd = 19 + vertexCount * 2
        if (vertexCount < 3 || values.size < polygonEnd + 2) return null
        val outlineVertices = ArrayList<FloorPlanPixelPoint>(vertexCount)
        repeat(vertexCount) { index ->
            val offset = 19 + index * 2
            outlineVertices += FloorPlanPixelPoint(
                x = values[offset].toFloat(),
                y = values[offset + 1].toFloat()
            )
        }
        val info = FloorPlanGenerationInfo(
            outlineClosed = values[0] >= 0.5,
            outlineWidthPixels = values[1].toFloat(),
            outlineHeightPixels = values[2].toFloat(),
            outlineLeftPixels = values[3].toFloat(),
            outlineTopPixels = values[4].toFloat(),
            outlineRightPixels = values[5].toFloat(),
            outlineBottomPixels = values[6].toFloat(),
            rotationDegrees = values[7].toFloat(),
            supportRatio = values[8].toFloat(),
            vertexCount = vertexCount,
            closeSizePixels = values[10].toInt(),
            dimensionCenterXPixels = values[11].toFloat(),
            dimensionCenterYPixels = values[12].toFloat(),
            dimensionLongAxisX = values[13].toFloat(),
            dimensionLongAxisY = values[14].toFloat(),
            dimensionShortAxisX = values[15].toFloat(),
            dimensionShortAxisY = values[16].toFloat(),
            dimensionLongSizePixels = values[17].toFloat(),
            dimensionShortSizePixels = values[18].toFloat(),
            outlineVerticesPixels = outlineVertices,
            footprintAreaPixelsSquared = values[polygonEnd].toFloat(),
            footprintPerimeterPixels = values[polygonEnd + 1].toFloat()
        )
        return info.takeIf {
            output.exists() && it.outlineWidthPixels > 0f && it.outlineHeightPixels > 0f &&
                it.outlineRightPixels > it.outlineLeftPixels &&
                it.outlineBottomPixels > it.outlineTopPixels &&
                it.dimensionLongSizePixels > 0f && it.dimensionShortSizePixels > 0f &&
                it.footprintAreaPixelsSquared > 0f && it.footprintPerimeterPixels > 0f
        }
    }

    private external fun nativeGenerateFloorPlan(
        inputPath: String,
        visualInputPath: String,
        semanticInputPath: String,
        outputPath: String,
        workDir: String,
        metersPerPixel: Double,
        trajectoryPixels: DoubleArray
    ): DoubleArray?
}
