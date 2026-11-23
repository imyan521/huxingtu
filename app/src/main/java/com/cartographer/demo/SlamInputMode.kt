package com.cartographer.demo

enum class SlamInputMode(val usesImu: Boolean, val displayName: String) {
    LIDAR_ONLY(false, "纯雷达建图"),
    LIDAR_IMU(true, "雷达 + IMU 建图");

    fun configurationFile(continuing: Boolean): String = when (this) {
        LIDAR_ONLY -> if (continuing) "android_2d_lidar_only_continue.lua" else "android_2d_lidar_only.lua"
        LIDAR_IMU -> if (continuing) "android_2d_continue.lua" else "android_2d.lua"
    }

    companion object {
        const val INTENT_EXTRA = "com.cartographer.sdk.INPUT_MODE"
        fun fromName(name: String?): SlamInputMode =
            values().firstOrNull { it.name == name } ?: LIDAR_IMU
    }
}
