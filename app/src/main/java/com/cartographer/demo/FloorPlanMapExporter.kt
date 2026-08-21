package com.cartographer.demo

import android.graphics.Bitmap
import android.graphics.Color
import kotlin.math.roundToInt

object FloorPlanMapExporter {

    // Match the neutral unknown-space canvas used by conventional occupancy
    // maps.  Keep the live SLAM view's dark/highlight palette independent.
    private const val UNKNOWN_COLOR = 0xFF9A9A9A.toInt()
    private const val FREE_COLOR = Color.WHITE
    private const val WALL_COLOR = Color.BLACK
    const val STABLE_WALL_MIN_ALPHA = 180

    enum class Style {
        // 给 1wall/户型图算法吃的输入图：白底 + 稳定黑墙。不要灰底、不要射线、不要轨迹。
        ALGORITHM_INPUT,

        // 最终户型图底图：竞品式占据栅格。未知区灰、已探索自由区白、
        // 稳定墙体黑；不是退化后的白底墙体骨架。
        VISUAL_MAP,

        // 拓扑重建输入：灰色未知区 + 白色空闲区 + 黑色稳定墙体。
        SEMANTIC,

        // 给人看的预览图：灰色未知区 + 白色已探索区 + 黑色墙体。
        PREVIEW
    }

    data class ExportGeometry(
        val resolutionMetersPerPixel: Float,
        val worldMinX: Float,
        val worldMaxX: Float,
        val worldMinY: Float,
        val worldMaxY: Float,
        val contentMinX: Float,
        val contentMaxX: Float,
        val contentMinY: Float,
        val contentMaxY: Float,
        val widthPx: Int,
        val heightPx: Int,
        val paddingCells: Int
    ) {
        fun worldToPixelX(worldX: Float): Float =
            (worldX - worldMinX) / resolutionMetersPerPixel

        fun worldToPixelY(worldY: Float): Float =
            (worldMaxY - worldY) / resolutionMetersPerPixel
    }

    data class RenderResult(
        val bitmap: Bitmap,
        val geometry: ExportGeometry
    )

    /**
     * Produces the floor-plan rasters from the exact finalized confidence grid
     * shown by [FusedMapRenderer]. All styles therefore share one pixel space
     * and one wall decision; only their presentation differs.
     */
    fun renderFusedWithGeometry(
        fused: FusedMapRenderer.Result,
        style: Style
    ): RenderResult? {
        val width = fused.bitmap.width
        val height = fused.bitmap.height
        val cellCount = width * height
        if (width <= 0 || height <= 0 ||
            fused.stableWallMask.size != cellCount ||
            fused.visibleWallMask.size != cellCount ||
            fused.freeSpaceMask.size != cellCount ||
            fused.occupancyPixels.size != cellCount ||
            !fused.resolutionMetersPerPixel.isFinite() ||
            fused.resolutionMetersPerPixel <= 0f) {
            return null
        }

        val occupancyStyle = style == Style.VISUAL_MAP ||
            style == Style.SEMANTIC || style == Style.PREVIEW
        // Structural extraction uses the same gap-closed, one-cell wall core
        // as the offline pbstream exporter. Keeping this decision shared makes
        // Android and desktop feed equivalent topology into floor_plan.cpp.
        val wallMask = if (style == Style.ALGORITHM_INPUT) {
            fused.stableWallMask
        } else {
            fused.visibleWallMask
        }
        val background = if (occupancyStyle) UNKNOWN_COLOR else Color.WHITE
        val pixels = if (style == Style.VISUAL_MAP) {
            fused.occupancyPixels.copyOf()
        } else {
            IntArray(cellCount) { background }
        }
        for (index in 0 until cellCount) {
            if (style == Style.VISUAL_MAP) continue
            pixels[index] = when {
                wallMask[index] -> WALL_COLOR
                occupancyStyle && fused.freeSpaceMask[index] -> FREE_COLOR
                else -> background
            }
        }

        val resolution = fused.resolutionMetersPerPixel
        val worldMaxX = fused.worldMinX + width * resolution
        val worldMinY = fused.worldMaxY - height * resolution
        val padding = ((fused.contentMinX - fused.worldMinX) / resolution)
            .roundToInt()
            .coerceAtLeast(0)
        return RenderResult(
            bitmap = Bitmap.createBitmap(
                pixels,
                width,
                height,
                Bitmap.Config.ARGB_8888
            ),
            geometry = ExportGeometry(
                resolutionMetersPerPixel = resolution,
                worldMinX = fused.worldMinX,
                worldMaxX = worldMaxX,
                worldMinY = worldMinY,
                worldMaxY = fused.worldMaxY,
                contentMinX = fused.contentMinX,
                contentMaxX = fused.contentMaxX,
                contentMinY = fused.contentMinY,
                contentMaxY = fused.contentMaxY,
                widthPx = width,
                heightPx = height,
                paddingCells = padding
            )
        )
    }

}
