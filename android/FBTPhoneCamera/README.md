# FBT Phone Camera

Small Android CameraX sender for the bodytracker network camera source.

It captures the selected front/back camera through CameraX ImageAnalysis, compresses each latest frame to JPEG, and pushes it over a raw TCP socket to the PC. The sender drops old queued frames instead of building latency. The backend is expected to listen with `camera_a.source = "network_mjpeg"` and `camera_a.network_port = 39555`.

## Build

Open this folder in Android Studio and run the `app` target. The project targets Android 16 / API 36 and uses CameraX 1.6.1.

Command line:

```bash
./gradlew assembleDebug
```

If you do not have a Gradle wrapper yet, let Android Studio create it or run with an installed Gradle compatible with Android Gradle Plugin 9.2.1.

## Use

1. Start bodytracker on the PC with `camera_a.source = "network_mjpeg"`.
2. Put phone and PC on the same LAN/hotspot.
3. Enter the PC's LAN IP in the app.
4. Leave the port at `39555` unless the backend config says otherwise.
5. Choose front or back camera.
6. Press **Start stream**.
