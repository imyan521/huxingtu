package customer.example

import android.os.Bundle
import androidx.appcompat.app.AppCompatActivity
import com.cartographer.sdk.CartographerListener
import com.cartographer.sdk.CartographerSdk
import com.cartographer.sdk.ConnectionState

class MinimalActivity : AppCompatActivity() {
    private lateinit var sdk: CartographerSdk

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        sdk = CartographerSdk.initialize(applicationContext, listener = object : CartographerListener {
            override fun onUsbPermissionResult(granted: Boolean) {
                if (granted) sdk.connect()
            }

            override fun onConnectionStateChanged(state: ConnectionState) {
                if (state == ConnectionState.CONNECTED) sdk.startMapping()
            }
        })
        sdk.requestUsbPermission(this)
    }

    override fun onDestroy() {
        sdk.close()
        super.onDestroy()
    }
}
