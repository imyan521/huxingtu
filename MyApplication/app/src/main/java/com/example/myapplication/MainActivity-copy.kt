package com.example.myapplication

import android.os.Bundle
import android.util.Log
import androidx.appcompat.app.AppCompatActivity
import java.io.File
import java.io.FileOutputStream

class `MainActivity-copy` : AppCompatActivity() {
    external fun runPipeline(inPath: String?, outPath: String?)

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        Log.d("TEST", "✅ 启动成功")

        val filesDir = getFilesDir()
        val input = File(filesDir, "map3.png").getAbsolutePath()
        val output = File(filesDir, "result.png").getAbsolutePath()

        // 从 assets 复制图片到私有目录（永远成功）
        try {
            val `is` = getAssets().open("map3.png")
            val fos = FileOutputStream(input)
            val buf = ByteArray(1024)
            var len: Int
            while ((`is`.read(buf).also { len = it }) > 0) fos.write(buf, 0, len)
            `is`.close()
            fos.close()
            Log.d("TEST", "✅ 图片复制成功！")
        } catch (e: Exception) {
            Log.e("TEST", "❌ 复制失败", e)
            return
        }

        // 运行算法
        Thread(Runnable {
            runPipeline(input, output)
            Log.d("TEST", "✅ 全部完成！去看结果！")
        }).start()
    }

    companion object {
        init {
            System.loadLibrary("wallpipeline")
            Log.d("TEST", "✅ 库加载成功")
        }
    }
}