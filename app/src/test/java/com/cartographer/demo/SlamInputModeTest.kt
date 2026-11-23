package com.cartographer.demo

import org.junit.Assert.*
import org.junit.Test

class SlamInputModeTest {
    @Test fun absentOrUnknownIntentKeepsPhoneImuDefault() {
        assertEquals(SlamInputMode.LIDAR_IMU, SlamInputMode.fromName(null))
        assertEquals(SlamInputMode.LIDAR_IMU, SlamInputMode.fromName("unknown"))
        assertTrue(SlamInputMode.fromName(null).usesImu)
    }

    @Test fun newAndSupplementMappingUseMatchingSensorProfiles() {
        val pure = SlamInputMode.fromName("LIDAR_ONLY")
        assertFalse(pure.usesImu)
        assertEquals("android_2d_lidar_only.lua", pure.configurationFile(false))
        assertEquals("android_2d_lidar_only_continue.lua", pure.configurationFile(true))
        assertEquals("android_2d.lua", SlamInputMode.LIDAR_IMU.configurationFile(false))
        assertEquals("android_2d_continue.lua", SlamInputMode.LIDAR_IMU.configurationFile(true))
    }
}
