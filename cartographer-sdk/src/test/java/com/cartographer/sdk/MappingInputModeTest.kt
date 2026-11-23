package com.cartographer.sdk

import org.junit.Assert.*
import org.junit.Test

class MappingInputModeTest {
    @Test fun existingConfigDefaultsAreUnchanged() {
        val config = SdkConfig()
        assertEquals(100L, config.snapshotIntervalMs)
        assertEquals(5_000_000L, config.imuMinimumIntervalNs)
        assertEquals(4107, config.permissionRequestCode)
        assertTrue(MappingInputMode.LIDAR_IMU.usesImu)
        assertFalse(MappingInputMode.LIDAR_ONLY.usesImu)
    }
}
