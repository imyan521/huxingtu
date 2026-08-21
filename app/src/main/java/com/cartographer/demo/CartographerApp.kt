package com.cartographer.demo

import android.app.Application

/**
 * Application 类
 *
 * 在此处初始化全局组件（如 Koin、Crashlytics 等）。
 * 目前保持轻量，后续可按需扩展。
 */
class CartographerApp : Application() {

    override fun onCreate() {
        super.onCreate()
        instance = this
    }

    companion object {
        lateinit var instance: CartographerApp
            private set
    }
}
