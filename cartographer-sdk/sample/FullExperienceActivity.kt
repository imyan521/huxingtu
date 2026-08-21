package customer.example

import android.os.Bundle
import android.widget.Button
import androidx.appcompat.app.AppCompatActivity
import com.cartographer.sdk.CartographerSdk

/** Customer-owned page that opens every packaged reference-app feature. */
class FullExperienceActivity : AppCompatActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(Button(this).apply {
            text = "打开雷达建图"
            setOnClickListener {
                startActivity(CartographerSdk.createFullExperienceIntent(this@FullExperienceActivity))
            }
        })
    }
}
