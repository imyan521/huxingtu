package com.cartographer.sdk

/** Input mode is fixed for each SDK instance. Existing initialize calls use LIDAR_IMU. */
enum class MappingInputMode(val usesImu: Boolean) {
    LIDAR_IMU(true),
    LIDAR_ONLY(false)
}
