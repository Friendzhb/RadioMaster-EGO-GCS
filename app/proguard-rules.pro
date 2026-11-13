-keep class okhttp3.** { *; }
-keep interface okhttp3.** { *; }
-dontwarn okhttp3.**
-dontwarn okio.**
-keep class androidx.core.content.FileProvider { *; }
-keep class com.radiomaster.egogcs.model.** { *; }
-keep class com.radiomaster.egogcs.ota.** { *; }
-keep class com.radiomaster.egogcs.engine.FlightTelemetryRecorder { *; }
-keepclassmembers class com.radiomaster.egogcs.** {
    public <init>(...);
}
