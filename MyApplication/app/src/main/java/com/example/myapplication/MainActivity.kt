package com.example.myapplication

import android.content.Intent
import android.graphics.BitmapFactory
import android.net.Uri
import android.os.Bundle
import android.provider.MediaStore
import android.util.Log
import android.widget.Button
import android.widget.ImageView
import android.widget.LinearLayout
import androidx.appcompat.app.AppCompatActivity
import java.io.File
import java.io.FileOutputStream

class MainActivity : AppCompatActivity() {

    companion object {
        init {
            System.loadLibrary("wallpipeline")
        }
    }

    external fun runPipeline(inPath: String, outPath: String)

    private val PICK_IMAGE = 100
    private lateinit var imageView: ImageView
    private lateinit var inputPath: String
    private lateinit var outputPath: String

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        val layout = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(50, 50, 50, 50)
        }

        imageView = ImageView(this).apply {
            layoutParams = LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                0,
                1.0f
            )
        }
        layout.addView(imageView)

        val selectBtn = Button(this).apply {
            text = "选择图片"
            setOnClickListener { openGallery() }
        }
        layout.addView(selectBtn)

        val processBtn = Button(this).apply {
            text = "开始处理"
            setOnClickListener { processImage() }
        }
        layout.addView(processBtn)

        setContentView(layout)

        val dir = filesDir
        inputPath = File(dir, "selected.png").absolutePath
        outputPath = File(dir, "result.png").absolutePath
    }

    private fun openGallery() {
        val intent = Intent(Intent.ACTION_PICK, MediaStore.Images.Media.EXTERNAL_CONTENT_URI)
        startActivityForResult(intent, PICK_IMAGE)
    }

    private fun processImage() {
        Thread {
            Log.d("TEST", "输入: $inputPath")
            Log.d("TEST", "输出: $outputPath")
            runPipeline(inputPath, outputPath)

            val bmp = BitmapFactory.decodeFile(outputPath)
            runOnUiThread { imageView.setImageBitmap(bmp) }
            Log.d("TEST", "✅ 处理完成！")
        }.start()
    }

    override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?) {
        super.onActivityResult(requestCode, resultCode, data)
        if (requestCode == PICK_IMAGE && resultCode == RESULT_OK && data != null) {
            try {
                val uri: Uri = data.data!!
                val inputStream = contentResolver.openInputStream(uri)
                val outputStream = FileOutputStream(inputPath)

                val buf = ByteArray(1024)
                var len: Int
                while (inputStream!!.read(buf).also { len = it } > 0) {
                    outputStream.write(buf, 0, len)
                }
                inputStream.close()
                outputStream.close()

                val bmp = BitmapFactory.decodeFile(inputPath)
                imageView.setImageBitmap(bmp)
                Log.d("TEST", "✅ 图片已加载")
            } catch (e: Exception) {
                e.printStackTrace()
            }
        }
    }
}
