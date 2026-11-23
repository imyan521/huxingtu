package consumer;
import android.content.Context;
import com.cartographer.sdk.*;
public class JavaClient {
    public static CartographerSdk original(Context context, CartographerListener listener) {
        return CartographerSdk.initialize(context, new SdkConfig(), listener);
    }
    public static CartographerSdk pure(Context context, CartographerListener listener) {
        return CartographerSdk.initialize(context, new SdkConfig(), MappingInputMode.LIDAR_ONLY, listener);
    }
    public static android.content.Intent page(Context context) {
        return CartographerSdk.createFullExperienceIntent(context, MappingInputMode.LIDAR_ONLY);
    }
}
