package customer.example

import android.os.Bundle
import androidx.appcompat.app.AppCompatActivity
import com.cartographer.sdk.CartographerSdk

/**
 * Minimal AAR-only integration that opens the complete Cartographer experience.
 *
 * The SDK-owned Activity provides the same mapping workflow and UI as the
 * source-built APK. This Activity does not create another CartographerSdk
 * instance, so there is no duplicate native engine or USB connection.
 */
class MinimalActivity : AppCompatActivity() {

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        if (savedInstanceState == null) {
            startActivity(CartographerSdk.createFullExperienceIntent(this))
        }
        finish()
    }
}
