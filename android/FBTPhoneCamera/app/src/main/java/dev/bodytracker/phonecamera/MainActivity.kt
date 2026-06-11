package dev.bodytracker.phonecamera

import android.Manifest
import android.content.Intent
import android.content.pm.PackageManager
import android.graphics.ImageFormat
import android.graphics.Rect
import android.graphics.YuvImage
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraManager
import android.os.Bundle
import android.os.SystemClock
import android.util.Log
import android.util.Size
import android.view.Gravity
import android.view.ViewGroup
import android.widget.Button
import android.widget.EditText
import android.widget.LinearLayout
import android.widget.TextView
import androidx.activity.ComponentActivity
import androidx.activity.result.contract.ActivityResultContracts
import androidx.camera.camera2.interop.Camera2CameraInfo
import androidx.camera.core.Camera
import androidx.camera.core.CameraInfo
import androidx.camera.core.CameraSelector
import androidx.camera.core.ImageAnalysis
import androidx.camera.core.ImageProxy
import androidx.camera.core.Preview
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.camera.view.PreviewView
import androidx.core.content.ContextCompat
import java.io.BufferedOutputStream
import java.io.ByteArrayOutputStream
import java.io.DataOutputStream
import java.io.IOException
import java.net.InetSocketAddress
import java.net.NetworkInterface
import java.net.Socket
import java.util.Collections
import java.util.concurrent.Executors
import java.util.concurrent.LinkedBlockingDeque
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicLong

class MainActivity : ComponentActivity() {
    private lateinit var previewView: PreviewView
    private lateinit var statusText: TextView
    private lateinit var pcIpEdit: EditText
    private lateinit var portEdit: EditText
    private lateinit var streamButton: Button
    private lateinit var findPcButton: Button
    private lateinit var cameraButton: Button
    private lateinit var qualityEdit: EditText

    private val analysisExecutor = Executors.newSingleThreadExecutor()
    private val scanExecutor = Executors.newSingleThreadExecutor()
    private val streamer = TcpMjpegStreamer { state ->
        runOnUiThread { statusText.text = state }
    }
    private val encodedFrames = AtomicLong(0)
    private val encoderErrors = AtomicLong(0)
    private var cameraMode = CameraMode.Back
    private var pendingAutoStart = false
    @Volatile private var jpegQuality = 72
    private val streaming = AtomicBoolean(false)

    private val permissionLauncher = registerForActivityResult(ActivityResultContracts.RequestPermission()) { granted ->
        if (granted) {
            bindCamera()
        } else {
            statusText.text = "Camera permission denied"
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        buildUi()
        applyConnectionIntent(intent)
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED) {
            bindCamera()
        } else {
            permissionLauncher.launch(Manifest.permission.CAMERA)
        }
    }

    override fun onDestroy() {
        streamer.stop()
        scanExecutor.shutdownNow()
        analysisExecutor.shutdownNow()
        super.onDestroy()
    }

    private fun buildUi() {
        previewView = PreviewView(this).apply {
            layoutParams = LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.MATCH_PARENT, 1f)
            scaleType = PreviewView.ScaleType.FIT_CENTER
        }

        statusText = TextView(this).apply {
            text = "Idle"
            textSize = 14f
        }
        pcIpEdit = EditText(this).apply {
            hint = "PC IP"
            setSingleLine(true)
            setText(savedText("pc_ip", defaultPcHost()))
        }
        portEdit = EditText(this).apply {
            hint = "Port"
            setSingleLine(true)
            setText(savedText("pc_port", "39555"))
            inputType = android.text.InputType.TYPE_CLASS_NUMBER
        }
        qualityEdit = EditText(this).apply {
            hint = "JPEG quality"
            setSingleLine(true)
            setText(savedText("jpeg_quality", "72"))
            inputType = android.text.InputType.TYPE_CLASS_NUMBER
        }
        streamButton = Button(this).apply {
            text = "Start stream"
            setOnClickListener { toggleStream() }
        }
        findPcButton = Button(this).apply {
            text = "Find PC"
            setOnClickListener { scanForPc() }
        }
        cameraButton = Button(this).apply {
            text = cameraMode.label
            setOnClickListener {
                cameraMode = cameraMode.next()
                text = cameraMode.label
                bindCamera()
            }
        }

        val controls = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            gravity = Gravity.TOP
            setPadding(24, 24, 24, 24)
            layoutParams = LinearLayout.LayoutParams(360, ViewGroup.LayoutParams.MATCH_PARENT)
            addView(TextView(this@MainActivity).apply {
                text = "FBT phone camera"
                textSize = 20f
            })
            addView(pcIpEdit)
            addView(portEdit)
            addView(qualityEdit)
            addView(findPcButton)
            addView(cameraButton)
            addView(streamButton)
            addView(statusText)
        }

        setContentView(LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            addView(previewView)
            addView(controls)
        })
    }

    override fun onNewIntent(intent: Intent) {
        super.onNewIntent(intent)
        setIntent(intent)
        applyConnectionIntent(intent)
    }

    private fun applyConnectionIntent(intent: Intent?) {
        if (intent == null) return
        val host = intent.getStringExtra("bt_host")?.trim().orEmpty()
        val port = intent.getIntExtra("bt_port", -1)
        val quality = intent.getIntExtra("bt_quality", -1)
        val autostart = intent.getBooleanExtra("bt_autostart", false)
        if (host.isNotBlank()) {
            pcIpEdit.setText(host)
        }
        if (port in 1..65535) {
            portEdit.setText(port.toString())
        }
        if (quality in 35..95) {
            qualityEdit.setText(quality.toString())
        }
        if (host.isNotBlank() || port in 1..65535 || quality in 35..95) {
            saveSettings(
                pcIpEdit.text.toString().trim(),
                portEdit.text.toString().trim().toIntOrNull() ?: 39555,
                qualityEdit.text.toString().trim().toIntOrNull()?.coerceIn(35, 95) ?: 72)
        }
        if (autostart) {
            pendingAutoStart = true
            streamButton.postDelayed({ maybeAutoStartStream() }, 750)
        }
    }

    private fun maybeAutoStartStream() {
        if (!pendingAutoStart || streaming.get()) return
        val host = pcIpEdit.text.toString().trim()
        if (host.isBlank()) return
        pendingAutoStart = false
        toggleStream()
    }

    private fun toggleStream() {
        if (streaming.get()) {
            streaming.set(false)
            streamer.stop()
            streamButton.text = "Start stream"
            statusText.text = "Stopped"
            return
        }

        val host = pcIpEdit.text.toString().trim()
        val port = portEdit.text.toString().trim().toIntOrNull() ?: 39555
        if (host.isBlank()) {
            statusText.text = "Enter the PC IP address"
            return
        }
        jpegQuality = qualityEdit.text.toString().trim().toIntOrNull()?.coerceIn(35, 95) ?: 72
        saveSettings(host, port, jpegQuality)
        streaming.set(true)
        streamer.start(host, port)
        streamButton.text = "Stop stream"
    }

    private fun scanForPc() {
        if (streaming.get()) return
        val port = portEdit.text.toString().trim().toIntOrNull() ?: 39555
        findPcButton.isEnabled = false
        statusText.text = "Scanning LAN for port $port"
        scanExecutor.execute {
            val localIp = localIpv4Address()
            val prefix = localIp?.substringBeforeLast('.')
            if (prefix == null) {
                runOnUiThread {
                    findPcButton.isEnabled = true
                    statusText.text = "No Wi-Fi IPv4 address found"
                }
                return@execute
            }

            val candidates = (1..254)
                .map { "$prefix.$it" }
                .filter { it != localIp }
                .sortedWith(compareBy<String> { it != "192.168.1.95" }.thenBy { it.substringAfterLast('.').toIntOrNull() ?: 999 })

            val found = candidates.firstOrNull { canConnect(it, port, 180) }
            runOnUiThread {
                findPcButton.isEnabled = true
                if (found != null) {
                    pcIpEdit.setText(found)
                    saveSettings(found, port, qualityEdit.text.toString().trim().toIntOrNull()?.coerceIn(35, 95) ?: 72)
                    statusText.text = "Found PC listener at $found:$port"
                } else {
                    statusText.text = "No PC listener found on $prefix.0/24:$port"
                }
            }
        }
    }

    private fun savedText(key: String, fallback: String): String {
        return getSharedPreferences("connection", MODE_PRIVATE).getString(key, fallback) ?: fallback
    }

    private fun saveSettings(host: String, port: Int, quality: Int) {
        getSharedPreferences("connection", MODE_PRIVATE).edit()
            .putString("pc_ip", host)
            .putString("pc_port", port.toString())
            .putString("jpeg_quality", quality.toString())
            .apply()
    }

    private fun bindCamera() {
        val providerFuture = ProcessCameraProvider.getInstance(this)
        providerFuture.addListener({
            val provider = providerFuture.get()
            val selectedBackId = if (cameraMode == CameraMode.UltraWide) selectBackCameraId(preferUltraWide = true) else null
            val selector = cameraSelectorForMode(cameraMode, selectedBackId)

            val preview = Preview.Builder().build().also {
                it.setSurfaceProvider(previewView.surfaceProvider)
            }

            val analysis = ImageAnalysis.Builder()
                .setTargetResolution(Size(1280, 720))
                .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                .setOutputImageFormat(ImageAnalysis.OUTPUT_IMAGE_FORMAT_YUV_420_888)
                .build()

            analysis.setAnalyzer(analysisExecutor) { image ->
                try {
                    if (streaming.get()) {
                        try {
                            val captureTimestampNanos = image.imageInfo.timestamp.takeIf { it > 0L }
                                ?: SystemClock.elapsedRealtimeNanos()
                            val jpeg = image.toJpeg(jpegQuality)
                            val encoded = encodedFrames.incrementAndGet()
                            val queued = streamer.offer(jpeg, captureTimestampNanos)
                            if (encoded <= 3L || encoded % 30L == 0L) {
                                val note = "Encoded frames=$encoded queued=${if (queued) "yes" else "replaced"} bytes=${jpeg.size}"
                                Log.i(TAG, note)
                                runOnUiThread { statusText.text = note }
                            }
                        } catch (e: Exception) {
                            val errors = encoderErrors.incrementAndGet()
                            val note = "Frame encode failed #$errors: ${e.message ?: e.javaClass.simpleName}"
                            Log.e(TAG, note, e)
                            if (errors <= 3L || errors % 30L == 0L) {
                                runOnUiThread { statusText.text = note }
                            }
                        }
                    }
                } finally {
                    image.close()
                }
            }

            try {
                provider.unbindAll()
                val camera = provider.bindToLifecycle(this, selector, preview, analysis)
                applyWideZoomIfAvailable(camera, cameraMode)
                statusText.text = cameraReadyText(cameraMode, selectedBackId, camera)
                streamButton.post { maybeAutoStartStream() }
            } catch (e: Exception) {
                statusText.text = "Camera bind failed: ${e.message}"
            }
        }, ContextCompat.getMainExecutor(this))
    }

    private fun cameraSelectorForMode(mode: CameraMode, cameraId: String?): CameraSelector {
        if (mode == CameraMode.Front) {
            return CameraSelector.DEFAULT_FRONT_CAMERA
        }
        if (mode == CameraMode.UltraWide && cameraId != null) {
            return CameraSelector.Builder()
                .requireLensFacing(CameraSelector.LENS_FACING_BACK)
                .addCameraFilter { infos: List<CameraInfo> ->
                    infos.filter { info ->
                        runCatching { Camera2CameraInfo.from(info).cameraId == cameraId }.getOrDefault(false)
                    }.ifEmpty { infos }
                }
                .build()
        }
        return CameraSelector.DEFAULT_BACK_CAMERA
    }

    private fun selectBackCameraId(preferUltraWide: Boolean): String? {
        val manager = getSystemService(CameraManager::class.java) ?: return null
        return manager.cameraIdList
            .mapNotNull { id ->
                val c = runCatching { manager.getCameraCharacteristics(id) }.getOrNull() ?: return@mapNotNull null
                if (c.get(CameraCharacteristics.LENS_FACING) != CameraCharacteristics.LENS_FACING_BACK) {
                    return@mapNotNull null
                }
                val focal = c.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS)
                    ?.filter { it.isFinite() && it > 0.0f }
                    ?.minOrNull() ?: return@mapNotNull null
                CameraChoice(id, focal)
            }
            .sortedBy { if (preferUltraWide) it.focalLengthMm else -it.focalLengthMm }
            .firstOrNull()
            ?.id
    }

    private fun applyWideZoomIfAvailable(camera: Camera, mode: CameraMode) {
        if (mode != CameraMode.UltraWide) return
        val minZoom = camera.cameraInfo.zoomState.value?.minZoomRatio ?: return
        if (minZoom.isFinite() && minZoom > 0.0f && minZoom < 1.0f) {
            camera.cameraControl.setZoomRatio(minZoom)
        }
    }

    private fun cameraReadyText(mode: CameraMode, selectedBackId: String?, camera: Camera): String {
        val cameraId = runCatching { Camera2CameraInfo.from(camera.cameraInfo).cameraId }.getOrNull()
        val minZoom = camera.cameraInfo.zoomState.value?.minZoomRatio
        val zoomNote = if (mode == CameraMode.UltraWide && minZoom != null && minZoom < 1.0f) {
            " minZoom=${"%.2f".format(minZoom)}"
        } else {
            ""
        }
        val idNote = cameraId ?: selectedBackId ?: "default"
        return "${mode.label} ready id=$idNote$zoomNote"
    }
}

private enum class CameraMode(val label: String) {
    UltraWide("Ultrawide camera"),
    Back("Back camera"),
    Front("Front camera");

    fun next(): CameraMode = when (this) {
        UltraWide -> Back
        Back -> Front
        Front -> UltraWide
    }
}

private data class CameraChoice(
    val id: String,
    val focalLengthMm: Float
)

private data class EncodedFrame(
    val jpeg: ByteArray,
    val timestampNanos: Long
)

private class TcpMjpegStreamer(
    private val onState: (String) -> Unit
) {
    private val running = AtomicBoolean(false)
    private val queue = LinkedBlockingDeque<EncodedFrame>(3)
    private val sent = AtomicLong(0)
    private val dropped = AtomicLong(0)
    private val sessionGeneration = AtomicLong(0)
    private val socketLock = Any()
    @Volatile private var thread: Thread? = null
    @Volatile private var activeSocket: Socket? = null

    fun start(host: String, port: Int) {
        stop()
        val session = sessionGeneration.incrementAndGet()
        sent.set(0)
        dropped.set(0)
        running.set(true)
        emitState(session, "Connecting to $host:$port")
        thread = Thread({ runLoop(session, host, port) }, "FBT-TCP-MJPEG-streamer-$session").also { it.start() }
    }

    fun stop() {
        running.set(false)
        sessionGeneration.incrementAndGet()
        closeActiveSocket()
        val oldThread = thread
        oldThread?.interrupt()
        if (oldThread != null && oldThread != Thread.currentThread()) {
            try {
                oldThread.join(500)
            } catch (_: InterruptedException) {
                Thread.currentThread().interrupt()
            }
        }
        if (thread === oldThread) {
            thread = null
        }
        queue.clear()
    }

    fun offer(jpeg: ByteArray, timestampNanos: Long): Boolean {
        if (!running.get()) return false
        if (jpeg.isEmpty()) return false
        if (!queue.offerLast(EncodedFrame(jpeg, timestampNanos))) {
            queue.pollFirst()
            dropped.incrementAndGet()
            queue.offerLast(EncodedFrame(jpeg, timestampNanos))
            return false
        }
        return true
    }

    private fun runLoop(session: Long, host: String, port: Int) {
        while (isCurrentSession(session)) {
            try {
                Socket().use { socket ->
                    setActiveSocket(session, socket)
                    try {
                        socket.tcpNoDelay = true
                        socket.connect(InetSocketAddress(host, port), 1500)
                        DataOutputStream(BufferedOutputStream(socket.getOutputStream(), 256 * 1024)).use { out ->
                            out.write("BTMJPEG1\n".toByteArray(Charsets.US_ASCII))
                            out.flush()
                            emitState(session, "Connected to $host:$port")
                            var emptyPolls = 0
                            while (isCurrentSession(session) && socket.isConnected && !socket.isClosed) {
                                val frame = queue.pollFirst(500, java.util.concurrent.TimeUnit.MILLISECONDS)
                                if (frame == null) {
                                    emptyPolls += 1
                                    if (emptyPolls == 4 || emptyPolls % 20 == 0) {
                                        emitState(session, "Connected to $host:$port; waiting for camera frames")
                                    }
                                    continue
                                }
                                emptyPolls = 0
                                if (!isCurrentSession(session)) break
                                out.writeInt(frame.jpeg.size)
                                out.writeLong(frame.timestampNanos)
                                out.write(frame.jpeg)
                                out.flush()
                                val n = sent.incrementAndGet()
                                if (n % 15L == 0L) {
                                    emitState(session, "Streaming: sent=$n dropped=${dropped.get()} bytes=${frame.jpeg.size}")
                                }
                            }
                        }
                    } finally {
                        clearActiveSocket(session, socket)
                    }
                }
            } catch (e: InterruptedException) {
                return
            } catch (e: IOException) {
                if (isCurrentSession(session)) {
                    emitState(session, "Disconnected from $host:$port: ${e.message ?: "network error"}; retrying")
                    sleepQuietly(session, 500)
                }
            } catch (e: Exception) {
                if (isCurrentSession(session)) {
                    emitState(session, "Stream error to $host:$port: ${e.message ?: e.javaClass.simpleName}; retrying")
                    sleepQuietly(session, 500)
                }
            }
        }
    }

    private fun isCurrentSession(session: Long): Boolean {
        return running.get() && sessionGeneration.get() == session
    }

    private fun emitState(session: Long, state: String) {
        if (sessionGeneration.get() == session) {
            onState(state)
        }
    }

    private fun setActiveSocket(session: Long, socket: Socket) {
        synchronized(socketLock) {
            if (sessionGeneration.get() == session) {
                activeSocket = socket
            }
        }
    }

    private fun clearActiveSocket(session: Long, socket: Socket) {
        synchronized(socketLock) {
            if (sessionGeneration.get() == session && activeSocket === socket) {
                activeSocket = null
            }
        }
    }

    private fun closeActiveSocket() {
        val socket = synchronized(socketLock) {
            activeSocket.also { activeSocket = null }
        }
        try {
            socket?.close()
        } catch (_: IOException) {
            // Closing is only a wake-up mechanism for connect/write/poll. Ignore close errors.
        }
    }

    private fun sleepQuietly(session: Long, ms: Long) {
        val deadline = SystemClock.elapsedRealtime() + ms
        while (isCurrentSession(session) && SystemClock.elapsedRealtime() < deadline) {
            try {
                Thread.sleep(50)
            } catch (_: InterruptedException) {
                Thread.currentThread().interrupt()
                return
            }
        }
    }
}

private fun defaultPcHost(): String = "127.0.0.1"

private const val TAG = "FBTPhoneCamera"

private fun localIpv4Address(): String? {
    return Collections.list(NetworkInterface.getNetworkInterfaces())
        .asSequence()
        .filter { it.isUp && !it.isLoopback }
        .flatMap { Collections.list(it.inetAddresses).asSequence() }
        .map { it.hostAddress ?: "" }
        .firstOrNull { it.matches(Regex("""\d+\.\d+\.\d+\.\d+""")) && !it.startsWith("127.") && !it.startsWith("169.254.") }
}

private fun canConnect(host: String, port: Int, timeoutMs: Int): Boolean {
    return try {
        Socket().use { socket ->
            socket.connect(InetSocketAddress(host, port), timeoutMs)
            true
        }
    } catch (_: IOException) {
        false
    } catch (_: SecurityException) {
        false
    }
}

private fun ImageProxy.toJpeg(quality: Int): ByteArray {
    val nv21 = yuv420888ToNv21(this)
    val out = ByteArrayOutputStream(nv21.size / 3)
    val yuv = YuvImage(nv21, ImageFormat.NV21, width, height, null)
    yuv.compressToJpeg(Rect(0, 0, width, height), quality, out)
    return out.toByteArray()
}

private fun yuv420888ToNv21(image: ImageProxy): ByteArray {
    val width = image.width
    val height = image.height
    val yPlane = image.planes[0]
    val uPlane = image.planes[1]
    val vPlane = image.planes[2]

    val out = ByteArray(width * height + width * height / 2)
    var offset = 0

    val yBuffer = yPlane.buffer
    val yRowStride = yPlane.rowStride
    val yPixelStride = yPlane.pixelStride
    for (row in 0 until height) {
        val rowStart = row * yRowStride
        for (col in 0 until width) {
            out[offset++] = yBuffer.get(rowStart + col * yPixelStride)
        }
    }

    val uBuffer = uPlane.buffer
    val vBuffer = vPlane.buffer
    val uRowStride = uPlane.rowStride
    val vRowStride = vPlane.rowStride
    val uPixelStride = uPlane.pixelStride
    val vPixelStride = vPlane.pixelStride
    val chromaHeight = height / 2
    val chromaWidth = width / 2

    for (row in 0 until chromaHeight) {
        for (col in 0 until chromaWidth) {
            val vIndex = row * vRowStride + col * vPixelStride
            val uIndex = row * uRowStride + col * uPixelStride
            out[offset++] = vBuffer.get(vIndex.coerceAtMost(vBuffer.limit() - 1))
            out[offset++] = uBuffer.get(uIndex.coerceAtMost(uBuffer.limit() - 1))
        }
    }
    return out
}
