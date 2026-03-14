package com.trickl.iosloggerarcore

import android.content.Context
import android.util.Size
import androidx.lifecycle.LifecycleOwner
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.coroutines.suspendCancellableCoroutine
import java.io.File
import android.view.Surface
import android.view.SurfaceHolder
import android.view.SurfaceView
import kotlin.coroutines.resume

class CaptureController(
    private val context: Context,
    private val lifecycleOwner: LifecycleOwner,
) {
    private companion object {
        // Diagnostic artifact only. Keep disabled for normal captures.
        private const val ENABLE_RAW_POSE_OUTPUT = true
    }

    private val poseMode = SharedArCoreCaptureEngine.PoseMode.LEGACY_STITCH

    private val scope = CoroutineScope(SupervisorJob() + Dispatchers.Main)

    private var datasetWriter: DatasetWriter? = null
    private var motionLocationSampler: MotionLocationSampler? = null
    private var sharedEngine: SharedArCoreCaptureEngine? = null
    private var idlePreviewEngine: IdleCameraPreviewEngine? = null
    private var bytesTickerJob: Job? = null
    private var previewSurface: Surface? = null
    private var bootToUnixOffsetSeconds: Double = 0.0
    private var latestPreviewView: SurfaceView? = null
    private var latestResolution: Size? = null
    private var hasCameraPermission: Boolean = false

    private val _isRecording = MutableStateFlow(false)
    val isRecording: StateFlow<Boolean> = _isRecording.asStateFlow()

    private val _status = MutableStateFlow("Ready")
    val status: StateFlow<String> = _status.asStateFlow()

    private val _trackingState = MutableStateFlow("IDLE")
    val trackingState: StateFlow<String> = _trackingState.asStateFlow()

    private val _recordedBytes = MutableStateFlow(0L)
    val recordedBytes: StateFlow<Long> = _recordedBytes.asStateFlow()

    private val _datasetPath = MutableStateFlow<String?>(null)
    val datasetPath: StateFlow<String?> = _datasetPath.asStateFlow()

    fun start(resolution: Size, previewView: SurfaceView) {
        if (_isRecording.value) return
        scope.launch {
            try {
                _status.value = "Starting..."
                latestPreviewView = previewView
                latestResolution = resolution

                stopIdlePreview()

                val baseDir = File(context.getExternalFilesDir(null), "datasets")
                datasetWriter = DatasetWriter(
                    baseDir = baseDir,
                    enableRawPoseOutput = ENABLE_RAW_POSE_OUTPUT,
                ).also { it.open() }
                _datasetPath.value = datasetWriter?.datasetDir?.absolutePath
                _recordedBytes.value = 0L

                val startButtonUnixSeconds = System.currentTimeMillis() / 1000.0
                datasetWriter?.writeEvent(
                    tsUnixSeconds = startButtonUnixSeconds,
                    event = "start_button_pressed",
                    details = "resolution=${resolution.width}x${resolution.height}"
                )

                bootToUnixOffsetSeconds = TimeUtils.bootToUnixOffsetSeconds()
                datasetWriter?.writeEvent(
                    tsUnixSeconds = startButtonUnixSeconds,
                    event = "boot_to_unix_offset_seconds",
                    details = "offset=$bootToUnixOffsetSeconds"
                )
                motionLocationSampler = MotionLocationSampler(
                    context = context,
                    bootToUnixOffsetSeconds = bootToUnixOffsetSeconds,
                    writerProvider = { datasetWriter }
                ).also { it.start() }

                _status.value = "Starting (ARCore SharedCamera)..."
                _trackingState.value = "STARTING"

                previewSurface = awaitPreviewSurface(previewView, resolution)

                val engine = SharedArCoreCaptureEngine(
                    context = context,
                    resolution = resolution,
                    outputVideoFile = datasetWriter!!.videoFile,
                    previewSurface = previewSurface,
                    bootToUnixOffsetSeconds = bootToUnixOffsetSeconds,
                    datasetWriter = datasetWriter!!,
                    onStatus = { msg -> _status.value = msg },
                    onTrackingState = { state -> _trackingState.value = state },
                    onStarted = {
                        _isRecording.value = true
                        startBytesTicker()
                    },
                    onStopped = { reason ->
                        stop(reason)
                    },
                    poseMode = poseMode,
                )

                sharedEngine = engine
                val started = engine.start()
                if (!started) {
                    stop("shared camera start failed")
                }
            } catch (t: Throwable) {
                _status.value = "Start failed: ${t.message ?: t::class.java.simpleName}"
                _trackingState.value = "ERROR"
                stop("start exception")
            }
        }
    }

    fun stop(reason: String? = null) {
        bytesTickerJob?.cancel()
        bytesTickerJob = null

        sharedEngine?.stop(reason)
        sharedEngine = null

        _isRecording.value = false

        motionLocationSampler?.stop()
        motionLocationSampler = null

        datasetWriter?.close()
        datasetWriter = null

        previewSurface?.release()
        previewSurface = null

        if (reason != null && reason.isNotBlank()) {
            _status.value = "Saved (stopped: $reason): ${_datasetPath.value}"
            _trackingState.value = "STOPPED"
        } else if (_datasetPath.value != null) {
            _status.value = "Saved: ${_datasetPath.value}"
            _trackingState.value = "STOPPED"
        }

        ensureIdlePreviewIfNeeded()
    }

    fun release() {
        stop("release")
        stopIdlePreview()
    }

    fun updatePreview(previewView: SurfaceView, resolution: Size, hasPermission: Boolean) {
        latestPreviewView = previewView
        latestResolution = resolution
        hasCameraPermission = hasPermission

        if (hasPermission) {
            if (!_isRecording.value) {
                refreshPreviewSurfaceGeometry(previewView, resolution)
            }
        }

        if (_isRecording.value || !hasPermission) {
            stopIdlePreview()
            return
        }

        ensureIdlePreviewIfNeeded()
    }

    private fun startBytesTicker() {
        bytesTickerJob?.cancel()
        bytesTickerJob = scope.launch {
            while (isActive && _isRecording.value) {
                _recordedBytes.value = datasetWriter?.videoFile?.length() ?: 0L
                delay(1000)
            }
        }
    }

    private suspend fun awaitPreviewSurface(surfaceView: SurfaceView, resolution: Size): Surface {
        refreshPreviewSurfaceGeometry(surfaceView, resolution)

        val holder = surfaceView.holder
        if (holder.surface.isValid) {
            return holder.surface
        }

        return suspendCancellableCoroutine { cont ->
            val callback = object : SurfaceHolder.Callback {
                override fun surfaceCreated(holder: SurfaceHolder) {
                    holder.removeCallback(this)
                    cont.resume(holder.surface)
                }

                override fun surfaceChanged(holder: SurfaceHolder, format: Int, width: Int, height: Int) = Unit

                override fun surfaceDestroyed(holder: SurfaceHolder) = Unit
            }

            holder.addCallback(callback)
            cont.invokeOnCancellation {
                holder.removeCallback(callback)
            }
        }
    }

    private fun refreshPreviewSurfaceGeometry(surfaceView: SurfaceView, resolution: Size) {
        surfaceView.holder.setFixedSize(resolution.width, resolution.height)
    }

    private fun ensureIdlePreviewIfNeeded() {
        val previewView = latestPreviewView ?: return
        val resolution = latestResolution ?: return
        if (!hasCameraPermission || _isRecording.value) return
        if (idlePreviewEngine != null) return

        scope.launch {
            try {
                val surface = awaitPreviewSurface(previewView, resolution)
                if (_isRecording.value || !hasCameraPermission) return@launch
                if (idlePreviewEngine != null) return@launch

                idlePreviewEngine = IdleCameraPreviewEngine(
                    context = context,
                    resolution = resolution,
                    previewSurface = surface,
                    onStatus = { msg ->
                        if (!_isRecording.value) {
                            _status.value = msg
                        }
                    }
                ).also { engine ->
                    if (!engine.start()) {
                        engine.stop()
                        idlePreviewEngine = null
                    } else if (!_isRecording.value) {
                        _status.value = "Preview ready"
                    }
                }
            } catch (_: Throwable) {
                // Ignore preview attach failures; recording start path will still handle preview setup.
            }
        }
    }

    private fun stopIdlePreview() {
        idlePreviewEngine?.stop()
        idlePreviewEngine = null
    }
}
