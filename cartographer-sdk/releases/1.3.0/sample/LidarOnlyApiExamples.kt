package consumer
import android.content.Context
import com.cartographer.sdk.*

// Compile against the minified delivery AAR, rather than SDK project classes.
fun originalDefaults(context: Context, listener: CartographerListener) =
    CartographerSdk.initialize(context, listener = listener)
fun originalExplicit(context: Context, listener: CartographerListener) =
    CartographerSdk.initialize(context, SdkConfig(100L, 5_000_000L, 4107), listener)
fun pureLidar(context: Context, listener: CartographerListener) =
    CartographerSdk.initialize(context, SdkConfig(), MappingInputMode.LIDAR_ONLY, listener)
fun originalPage(context: Context) = CartographerSdk.createFullExperienceIntent(context)
fun purePage(context: Context) = CartographerSdk.createFullExperienceIntent(context, MappingInputMode.LIDAR_ONLY)
fun callbacksAndMaps(sdk: CartographerSdk, file: java.io.File) {
    sdk.getFinalRasterMap { result, map -> if (result.isSuccess) map?.bitmap?.recycle() }
    sdk.getTrajectoryNodePoseSamples { samples -> samples.forEach { it.timestampNs } }
    sdk.saveMap(file) { it.isSuccess }
    sdk.loadMap(file) { it.isSuccess }
}
