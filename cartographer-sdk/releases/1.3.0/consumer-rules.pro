# Only the stable SDK surface is kept in the consuming application.
-keep public class com.cartographer.sdk.** { public protected *; }
-keep class com.cartographer.demo.CartographerNative { native <methods>; }
-keep class com.cartographer.demo.FloorPlanNative { native <methods>; }
-keep class com.cartographer.demo.MainActivity { *; }
-keep class com.cartographer.sdk.FullExperienceActivity { *; }
