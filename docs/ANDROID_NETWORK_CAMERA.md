# Android network camera source

This patch adds a phone camera path without making the phone a second-class camera.

The Android app streams raw TCP MJPEG frames to the PC. The C++ backend listens on a configured local port, decodes the JPEG frames with OpenCV, and stores them in the same `FrameSlot` used by local OpenCV cameras. Downstream tracking sees pixels, timestamps, sequence numbers, and frame age. It does not get an `is_network` excuse to veto the data.

## Protocol

The Android app opens a TCP socket to the PC and writes:

```text
BTMJPEG1\n
[uint32 big-endian jpeg_size]
[uint64 big-endian phone_elapsed_realtime_nanos]
[jpeg bytes]
...
```

There is no TLS and no auth. This is for a trusted local LAN / hotspot. The port is whatever `camera_a.network_port` or `camera_b.network_port` says.

## Backend config

For a phone as the single live camera:

```json
"tracking": {
  "mode": "monocular"
},
"camera_a": {
  "source": "network_mjpeg",
  "network_bind_address": "0.0.0.0",
  "network_port": 39555,
  "network_read_timeout_ms": 1000,
  "network_max_frame_bytes": 8388608,
  "width": 640,
  "height": 480,
  "fps": 30
}
```

The width/height/fps fields are still useful as expected runtime settings and UI/config metadata. The actual decoded frame size is published from the stream.

For stereo with two phones, set `camera_b.source` to `network_mjpeg` too and use a different port, for example `39556`. The backend rejects two network cameras on the same bind address and port because that is not “stale,” it is physically the same socket.

## Android build

Open `android/FBTPhoneCamera` in Android Studio, sync, and run the `app` configuration on the phone.

Command line from that folder:

```powershell
.\gradlew.bat assembleDebug
```

or on macOS/Linux:

```bash
./gradlew assembleDebug
```

This source tree intentionally does not include a generated Gradle wrapper binary. Android Studio can create/use the wrapper, or you can run it with an installed Gradle matching the Android Gradle Plugin.

## Runtime steps

1. Put the phone and PC on the same LAN or hotspot.
2. Set `camera_a.source` to `network_mjpeg` and `camera_a.network_port` to `39555` in the bodytracker config.
3. Start `bodytracker.exe`. The camera health panel should show `network_mjpeg_tcp:0.0.0.0:39555` and wait for the phone stream.
4. Open the Android app, enter the PC IP address and port `39555`, select front/back camera, then press **Start stream**.
5. If Wi-Fi drops or stalls, the backend keeps the last finite frame in the slot. Frame age rises, pose/keypoint reliability is scaled down by age, and those pixels still enter the solver until there are no pixels at all. `prediction_only` is for missing frames, not merely old network frames.

## Failure semantics

Network receive stalls, disconnects, malformed JPEGs, and decode misses update capture health and keep the runtime alive. They do not clear the `FrameSlot`, do not mark the whole camera pipeline invalid, and do not halt tracking.

Only configuration impossibilities are rejected at config-load time: invalid source name, invalid port range, absurd frame-size bound, or two stereo network cameras bound to the exact same address and port.
