package com.trickl.iosloggerarcore

import android.Manifest
import android.app.Activity
import android.content.Context
import android.content.pm.PackageManager
import android.graphics.ImageFormat
import android.hardware.camera2.CameraCaptureSession
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraDevice
import android.hardware.camera2.CameraManager
import android.hardware.camera2.CaptureRequest
import android.media.ImageReader
import android.media.MediaRecorder
import android.os.Handler
import android.os.HandlerThread
import android.os.SystemClock
import android.opengl.EGL14
import android.opengl.EGLConfig
import android.opengl.EGLContext
import android.opengl.EGLDisplay
import android.opengl.EGLSurface
import android.opengl.GLES20
import android.util.Log
import android.util.Size
import android.view.Surface
import androidx.core.content.ContextCompat
import com.google.ar.core.Anchor
import com.google.ar.core.ArCoreApk
import com.google.ar.core.Config
import com.google.ar.core.Session
import com.google.ar.core.TrackingState
import com.google.ar.core.exceptions.CameraNotAvailableException
import java.io.File
import java.util.Locale
import java.util.EnumSet
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicInteger
import java.util.concurrent.atomic.AtomicLong
import kotlin.math.abs
import kotlin.math.acos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

class SharedArCoreCaptureEngine(
    private val context: Context,
    private val resolution: Size,
    private val outputVideoFile: File,
    private val previewSurface: Surface?,
    private val bootToUnixOffsetSeconds: Double,
    private val datasetWriter: DatasetWriter,
    private val onStatus: (String) -> Unit,
    private val onTrackingState: (String) -> Unit,
    private val onStarted: () -> Unit,
    private val onStopped: (String?) -> Unit,
    private val poseMode: PoseMode = PoseMode.LEGACY_STITCH,
) {
    private enum class TrackingPipelineState {
        WARMUP,
        VALID_RECORDING,
        INVALID_TRACKING_PAUSED,
        RELOCALIZATION_HOLD,
        RESUMED_VALID,
    }

    enum class PoseMode {
        LEGACY_STITCH,
        ANCHOR_RELATIVE,
    }

    private val tag = "SharedArCoreCapture"

    private val started = AtomicBoolean(false)
    private val stopping = AtomicBoolean(false)

    private var arSession: Session? = null
    private var sharedCamera: com.google.ar.core.SharedCamera? = null

    private var cameraDevice: CameraDevice? = null
    private var captureSession: CameraCaptureSession? = null
    private var captureRequest: CaptureRequest? = null

    private var mediaRecorder: MediaRecorder? = null
    private var imageReader: ImageReader? = null

    private var cameraThread: HandlerThread? = null
    private var cameraHandler: Handler? = null

    private var eglDisplay: EGLDisplay? = null
    private var eglContext: EGLContext? = null
    private var eglSurface: EGLSurface? = null
    private var cameraTextureId: Int = -1

    private val sourceFrameCounter = AtomicLong(0)
    private val frameCounter = AtomicLong(0)
    private val missingPoseCounter = AtomicInteger(0)
    private val updateErrorCounter = AtomicInteger(0)
    private val adjustedPoseTimestampCounter = AtomicInteger(0)
    private val relocalizationCounter = AtomicInteger(0)
    private var recordingStartElapsedNanos: Long = 0L
    private var hadTracking: Boolean = false
    private var lastPoseTimestampSeconds: Double = Double.NEGATIVE_INFINITY
    private var previousTrackingState: TrackingState? = null
    private var pendingRelocalization: Boolean = false
    private var previousRawArKitPose: PoseSample? = null
    private var previousExportedPoseWorld: PoseSample? = null
    private var previousExportedPoseRelative: PoseSample? = null
    private var firstReferenceWorldPose: PoseSample? = null
    private var previousSmoothedPoseRelative: PoseSample? = null
    private var worldReanchorTransform: RigidTransform? = null
    private var captureAnchor: Anchor? = null
    private var anchorComparisonDisabled: Boolean = false
    private var firstAnchorEventWritten: Boolean = false
    private var firstFrameEventWritten: Boolean = false
    private var firstPoseEventWritten: Boolean = false
    private var trackingPipelineState: TrackingPipelineState = TrackingPipelineState.WARMUP
    private var trackingConsecutiveFrames: Int = 0
    private var recoveringConsecutiveFrames: Int = 0
    private var previousAnchorComparisonPose: PoseSample? = null
    private var anchorHealthyStreak: Int = 0
    private var anchorUnhealthyStreak: Int = 0
    private var anchorCreateCooldownFrames: Int = 0
    private var relocalizationHoldFramesRemaining: Int = 0
    private var mediaRecorderPaused: Boolean = false

    private data class RigidTransform(
        val tx: Double,
        val ty: Double,
        val tz: Double,
        val qw: Double,
        val qx: Double,
        val qy: Double,
        val qz: Double,
    )

    private data class PoseDelta(
        val translationMeters: Double,
        val rotationDegrees: Double,
    )

    private data class TranslationDeltaBreakdown(
        val worldDx: Double,
        val worldDy: Double,
        val worldDz: Double,
        val cameraDx: Double,
        val cameraDy: Double,
        val cameraDz: Double,
    )

    private data class ExportPoseComponents(
        val qw: Double,
        val qx: Double,
        val qy: Double,
        val qz: Double,
    )

    private data class IntrinsicsSnapshot(
        val raw: CameraIntrinsics,
        val mapped: CameraIntrinsics,
        val srcWidth: Int,
        val srcHeight: Int,
    )

    private class ScalarKalmanFilter(
        private val processNoise: Double,
        private val measurementNoise: Double,
    ) {
        private var initialized = false
        private var estimate = 0.0
        private var covariance = 1.0

        fun reset() {
            initialized = false
            estimate = 0.0
            covariance = 1.0
        }

        fun update(measurement: Double): Double {
            if (!initialized) {
                estimate = measurement
                initialized = true
                return estimate
            }

            covariance += processNoise
            val gain = covariance / (covariance + measurementNoise)
            estimate += gain * (measurement - estimate)
            covariance = (1.0 - gain) * covariance
            return estimate
        }
    }

    private data class LagEstimate(
        val lagFrames: Int,
        val lagSeconds: Double,
        val correlation: Double,
    )

    private class PoseLagEstimator(
        private val windowSize: Int,
        private val maxLagFrames: Int,
    ) {
        private val rawSpeed = ArrayDeque<Double>()
        private val filteredSpeed = ArrayDeque<Double>()
        private val dtSeries = ArrayDeque<Double>()

        fun reset() {
            rawSpeed.clear()
            filteredSpeed.clear()
            dtSeries.clear()
        }

        fun addSample(rawStep: Double, filteredStep: Double, dtSeconds: Double) {
            push(rawSpeed, rawStep)
            push(filteredSpeed, filteredStep)
            if (dtSeconds.isFinite() && dtSeconds > 0.0) {
                push(dtSeries, dtSeconds)
            }
        }

        fun estimate(): LagEstimate? {
            val raw = rawSpeed.toList()
            val filtered = filteredSpeed.toList()
            if (raw.size < windowSize || filtered.size < windowSize) return null

            var bestLag = 0
            var bestCorr = Double.NEGATIVE_INFINITY

            for (lag in 0..maxLagFrames) {
                val corr = correlationAtLag(raw, filtered, lag)
                if (corr > bestCorr) {
                    bestCorr = corr
                    bestLag = lag
                }
            }

            val dtMedian = median(dtSeries.toList()).coerceAtLeast(0.0)
            return LagEstimate(
                lagFrames = bestLag,
                lagSeconds = dtMedian * bestLag,
                correlation = if (bestCorr.isFinite()) bestCorr else 0.0,
            )
        }

        private fun correlationAtLag(raw: List<Double>, filtered: List<Double>, lag: Int): Double {
            val n = min(raw.size, filtered.size)
            if (n - lag < 8) return Double.NEGATIVE_INFINITY
            val x = raw.subList(0, n - lag)
            val y = filtered.subList(lag, n)
            return pearsonCorrelation(x, y)
        }

        private fun pearsonCorrelation(x: List<Double>, y: List<Double>): Double {
            if (x.size != y.size || x.isEmpty()) return Double.NEGATIVE_INFINITY
            val n = x.size.toDouble()
            val mx = x.sum() / n
            val my = y.sum() / n

            var num = 0.0
            var dx2 = 0.0
            var dy2 = 0.0
            for (i in x.indices) {
                val dx = x[i] - mx
                val dy = y[i] - my
                num += dx * dy
                dx2 += dx * dx
                dy2 += dy * dy
            }

            val den = sqrt(dx2 * dy2)
            if (den <= 1e-12) return Double.NEGATIVE_INFINITY
            return num / den
        }

        private fun median(values: List<Double>): Double {
            if (values.isEmpty()) return 0.0
            val s = values.sorted()
            val mid = s.size / 2
            return if (s.size % 2 == 0) 0.5 * (s[mid - 1] + s[mid]) else s[mid]
        }

        private fun push(buffer: ArrayDeque<Double>, value: Double) {
            buffer.addLast(value)
            while (buffer.size > windowSize) {
                buffer.removeFirst()
            }
        }
    }

    private var lastIntrinsicsFingerprint: String? = null
    private val kalmanTx = ScalarKalmanFilter(processNoise = 1e-4, measurementNoise = 2e-3)
    private val kalmanTy = ScalarKalmanFilter(processNoise = 1e-4, measurementNoise = 2e-3)
    private val kalmanTz = ScalarKalmanFilter(processNoise = 1e-4, measurementNoise = 2e-3)
    private val lagEstimator = PoseLagEstimator(windowSize = 90, maxLagFrames = 12)
    private var previousRawRelativePoseForLag: PoseSample? = null
    private var previousFilteredPoseForLag: PoseSample? = null
    private var previousLagSampleTimestamp: Double? = null
    private var lastLoggedLagFrames: Int = Int.MIN_VALUE

    private companion object {
        private const val FIRST_PRINCIPLES_POSE_VALIDATION_MODE = false
        private const val ENABLE_CONTINUITY_REANCHOR = !FIRST_PRINCIPLES_POSE_VALIDATION_MODE
        private const val ENABLE_ANCHOR_COMPARISON = !FIRST_PRINCIPLES_POSE_VALIDATION_MODE
        private const val ALLOW_JUMP_ONLY_REANCHOR = false
        private const val ENABLE_FIRST_FRAME_RELATIVE_EXPORT = true
        private const val ENABLE_KALMAN_FILTER = true
        private const val ROTATION_SMOOTHING_ALPHA = 0.1

        private const val HARD_TRANSLATION_RESET_M = 0.20
        private const val HARD_ROTATION_RESET_DEG = 20.0
        private const val COMBINED_TRANSLATION_RESET_M = 0.12
        private const val COMBINED_ROTATION_RESET_DEG = 10.0
        // Resolution-agnostic hard safety: per-frame motion this large is physically
        // implausible for handheld capture and indicates world-frame discontinuity.
        private const val PHYSICALLY_IMPLAUSIBLE_TRANSLATION_JUMP_M = 0.50
        private const val PHYSICALLY_IMPLAUSIBLE_ROTATION_JUMP_DEG = 30.0
        private const val DISCONTINUITY_REJECT_TRANSLATION_M = 0.20
        private const val DISCONTINUITY_REJECT_ROTATION_DEG = 20.0
        private const val ACCEPTED_CONTINUITY_MAX_TRANSLATION_M = 0.20
        private const val ACCEPTED_CONTINUITY_MAX_ROTATION_DEG = 20.0
        private const val POSE_INSTRUMENTATION_LOG_EVERY_FRAME = true
        private const val STABLE_TRACKING_MIN_FRAMES = 12
        private const val RELOCALIZATION_HOLD_FRAMES = STABLE_TRACKING_MIN_FRAMES
        private const val RECOVERING_STOP_FRAMES = 120
        private const val ANCHOR_CREATE_MIN_STABLE_FRAMES = 18
        private const val ANCHOR_CREATE_COOLDOWN_FRAMES = 45
        private const val ANCHOR_MAX_UNHEALTHY_STREAK = 24
        private const val ANCHOR_MAX_FRAME_DELTA_TRANSLATION_M = 0.09
        private const val ANCHOR_MAX_FRAME_DELTA_ROTATION_DEG = 6.5
        private const val TRANSFORM_CHAIN_ID =
            "T_wc_arcore_raw -> B_arcore_to_arkit * T_wc * B^T -> C_cam_roll_z_m90 -> T_wc_arkit_raw -> T_world_reanchor -> T_wc_export"
    }

    fun start(): Boolean {
        val activity = context as? Activity ?: run {
            onStatus("Start failed: context is not Activity")
            return false
        }

        if (!ensureArCoreAvailable(activity)) {
            return false
        }

        return try {
            cameraThread = HandlerThread("shared-arcore-camera").also { it.start() }
            cameraHandler = Handler(cameraThread!!.looper)
            onTrackingState("STARTING")

            arSession = Session(
                activity,
                EnumSet.of(Session.Feature.SHARED_CAMERA)
            ).also { session ->
                val config = Config(session).apply {
                    updateMode = Config.UpdateMode.LATEST_CAMERA_IMAGE
                }
                session.configure(config)
            }

            runOnCameraThreadBlocking {
                initOffscreenGlContext()
                bindArCoreTexture()
            }

            sharedCamera = arSession!!.getSharedCamera()

            setupMediaRecorder()
            setupImageReader()
            openSharedCamera()
            Log.i(tag, "start(): Shared camera opening requested")

            true
        } catch (e: Throwable) {
            onStatus("Start failed: ${e.message ?: e::class.java.simpleName}")
            onTrackingState("ERROR")
            Log.e(tag, "start(): failed", e)
            stopInternal("start error")
            false
        }
    }

    fun stop(reason: String? = null) {
        stopInternal(reason)
    }

    private fun ensureArCoreAvailable(activity: Activity): Boolean {
        return try {
            val availability = ArCoreApk.getInstance().checkAvailability(activity)
            if (!availability.isSupported) {
                onStatus("Start failed: Device does not support ARCore")
                return false
            }
            val install = ArCoreApk.getInstance().requestInstall(activity, true)
            if (install == ArCoreApk.InstallStatus.INSTALL_REQUESTED) {
                onStatus("Start failed: ARCore install/update requested; try again after installation")
                return false
            }
            true
        } catch (e: Throwable) {
            onStatus("Start failed: ARCore availability check failed (${e.message ?: e::class.java.simpleName})")
            false
        }
    }

    private fun setupMediaRecorder() {
        mediaRecorder = MediaRecorder().apply {
            setVideoSource(MediaRecorder.VideoSource.SURFACE)
            setOutputFormat(MediaRecorder.OutputFormat.MPEG_4)
            setOutputFile(outputVideoFile.absolutePath)
            setVideoEncoder(MediaRecorder.VideoEncoder.H264)
            setVideoEncodingBitRate(10_000_000)
            setVideoFrameRate(30)
            setVideoSize(resolution.width, resolution.height)
            prepare()
        }
    }

    private fun setupImageReader() {
        imageReader = ImageReader.newInstance(
            resolution.width,
            resolution.height,
            ImageFormat.YUV_420_888,
            3,
        ).apply {
            setOnImageAvailableListener({ reader ->
                val image = reader.acquireLatestImage() ?: return@setOnImageAvailableListener
                try {
                    processFrame(image.timestamp)
                } catch (e: Throwable) {
                    onStatus("Frame processing error: ${e.message ?: e::class.java.simpleName}")
                    Log.e(tag, "setupImageReader(): frame processing error", e)
                    stopInternal("frame processing error")
                } finally {
                    image.close()
                }
            }, cameraHandler)
        }
    }

    private fun processFrame(timestampNanos: Long) {
        val session = arSession ?: return

        val ts = TimeUtils.toUnixSeconds(timestampNanos, bootToUnixOffsetSeconds)
        val sourceFrame = sourceFrameCounter.getAndIncrement()

        val arFrame = try {
            makeGlCurrent()
            session.update()
        } catch (e: CameraNotAvailableException) {
            onStatus("Stopped: ARCore camera unavailable (${e.message ?: e::class.java.simpleName})")
            Log.e(tag, "processFrame(): CameraNotAvailableException", e)
            stopInternal("arcore camera unavailable")
            return
        } catch (e: Throwable) {
            val errors = updateErrorCounter.incrementAndGet()
            pauseMediaRecorderIfActive(ts, "session_update_failed")
            onTrackingState("PAUSED")
            if (errors % 10 == 0) {
                Log.w(tag, "processFrame(): session.update failed $errors times (${e::class.java.simpleName}: ${e.message})")
            }
            if (errors > 600) {
                onStatus("Stopped: ARCore update repeatedly failed (${e::class.java.simpleName})")
                Log.e(tag, "processFrame(): stopping after repeated update failures", e)
                stopInternal("arcore update failure")
            }
            return
        }

        updateErrorCounter.set(0)

        val camera = arFrame.camera
        onTrackingState(camera.trackingState.name)

        val intrinsicsSnapshot = try {
            val imageIntrinsics = camera.imageIntrinsics
            val fxFy = imageIntrinsics.focalLength
            val cxCy = imageIntrinsics.principalPoint
            val srcDims = imageIntrinsics.imageDimensions
            val rawIntrinsics = CameraIntrinsics(
                fx = fxFy[0].toDouble(),
                fy = fxFy[1].toDouble(),
                cx = cxCy[0].toDouble(),
                cy = cxCy[1].toDouble(),
            )

            val mappedIntrinsics = mapIntrinsicsToOutputResolution(
                intrinsics = rawIntrinsics,
                srcWidth = srcDims[0],
                srcHeight = srcDims[1],
                dstWidth = resolution.width,
                dstHeight = resolution.height,
            )

            IntrinsicsSnapshot(
                raw = rawIntrinsics,
                mapped = mappedIntrinsics,
                srcWidth = srcDims[0],
                srcHeight = srcDims[1],
            )
        } catch (_: Throwable) {
            null
        }

        val intrinsics = intrinsicsSnapshot?.mapped
        if (intrinsicsSnapshot != null) {
            logIntrinsicsConsistency(
                tsUnixSeconds = ts,
                sourceFrameIndex = sourceFrame,
                snapshot = intrinsicsSnapshot,
            )
        }

        if (poseMode == PoseMode.ANCHOR_RELATIVE) {
            processFrameAnchorMode(
                session = session,
                ts = ts,
                sourceFrame = sourceFrame,
                camera = camera,
                intrinsics = intrinsics,
            )
        } else {
            processFrameLegacyMode(
                arFrameTimestampNanos = arFrame.timestamp,
                ts = ts,
                sourceFrame = sourceFrame,
                camera = camera,
                intrinsics = intrinsics,
            )
        }
    }

    private fun processFrameAnchorMode(
        session: Session,
        ts: Double,
        sourceFrame: Long,
        camera: com.google.ar.core.Camera,
        intrinsics: CameraIntrinsics?,
    ) {
        resumeMediaRecorderIfPaused(ts)

        if (camera.trackingState != TrackingState.TRACKING) {
            transitionTrackingPipelineState(TrackingPipelineState.INVALID_TRACKING_PAUSED, ts)
            pauseMediaRecorderIfActive(
                tsUnixSeconds = ts,
                reason = "anchor_mode_tracking_invalid:${camera.trackingState.name}"
            )
            val elapsedMs = if (recordingStartElapsedNanos == 0L) 0L
            else (SystemClock.elapsedRealtimeNanos() - recordingStartElapsedNanos) / 1_000_000L

            if (hadTracking) {
                onStatus("Paused: ARCore tracking lost (${camera.trackingState})")
                return
            }

            if (!hadTracking && elapsedMs < 10_000L) {
                onStatus("Waiting for ARCore tracking... (${camera.trackingState})")
                missingPoseCounter.set(0)
                return
            }

            val misses = missingPoseCounter.incrementAndGet()
            if (misses > 300) {
                onStatus("Paused: ARCore tracking not active (${camera.trackingState})")
            }
            return
        }

        val cameraPose = camera.pose
        if (captureAnchor == null) {
            captureAnchor = try {
                session.createAnchor(cameraPose)
            } catch (t: Throwable) {
                onStatus("Stopped: failed to create capture anchor (${t.message ?: t::class.java.simpleName})")
                Log.e(tag, "processFrame(): failed to create anchor", t)
                stopInternal("anchor create failure")
                return
            }

            if (!firstAnchorEventWritten) {
                datasetWriter.writeEvent(
                    tsUnixSeconds = ts,
                    event = "capture_anchor_created",
                    details = "sourceFrameIndex=$sourceFrame"
                )
                firstAnchorEventWritten = true
            }
        }

        val anchor = captureAnchor
        if (anchor == null || anchor.trackingState != TrackingState.TRACKING) {
            transitionTrackingPipelineState(TrackingPipelineState.INVALID_TRACKING_PAUSED, ts)
            pauseMediaRecorderIfActive(ts, "anchor_mode_anchor_invalid")
            if (hadTracking) {
                onStatus("Paused: capture anchor tracking lost")
                return
            }

            val misses = missingPoseCounter.incrementAndGet()
            onStatus("Waiting for capture anchor tracking...")
            if (misses > 300) {
                onStatus("Paused: capture anchor tracking not active")
            }
            return
        }

        hadTracking = true
        missingPoseCounter.set(0)
        transitionTrackingPipelineState(TrackingPipelineState.VALID_RECORDING, ts)
        resumeMediaRecorderIfPaused(ts)

        val frame = frameCounter.getAndIncrement()
        datasetWriter.writeFrame(ts, frame, intrinsics)

        if (!firstFrameEventWritten) {
            datasetWriter.writeEvent(
                tsUnixSeconds = ts,
                event = "first_frame_seen",
                details = "frameIndex=$frame,sourceFrameIndex=$sourceFrame"
            )
            firstFrameEventWritten = true
        }

        val poseRelativeToAnchor = anchor.pose.inverse().compose(cameraPose)
        val q = poseRelativeToAnchor.rotationQuaternion // x, y, z, w
        val poseTsRaw = ts
        val poseTs = makePoseTimestampStrictlyIncreasing(poseTsRaw)

        val exportedPose = ArKitConventionMapper.mapPose(
            timestampSeconds = poseTs,
            tx = poseRelativeToAnchor.tx().toDouble(),
            ty = poseRelativeToAnchor.ty().toDouble(),
            tz = poseRelativeToAnchor.tz().toDouble(),
            qx = q[0].toDouble(),
            qy = q[1].toDouble(),
            qz = q[2].toDouble(),
            qw = q[3].toDouble(),
        )

        val exportedPoseFixed = applyArposesUpAxisHalfTurnFix(exportedPose)
        datasetWriter.writePose(
            exportedPose.timestampSeconds,
            exportedPose.tx,
            exportedPose.ty,
            exportedPose.tz,
            exportedPoseFixed.qx,
            exportedPoseFixed.qy,
            exportedPoseFixed.qz,
            exportedPoseFixed.qw,
        )

        if (!firstPoseEventWritten) {
            datasetWriter.writeEvent(
                tsUnixSeconds = exportedPose.timestampSeconds,
                event = "first_pose_written",
                details = String.format(
                    Locale.US,
                    "frameIndex=%d,tx=%.6f,ty=%.6f,tz=%.6f,qx=%.6f,qy=%.6f,qz=%.6f,qw=%.6f",
                    frame,
                    exportedPose.tx,
                    exportedPose.ty,
                    exportedPose.tz,
                    exportedPose.qx,
                    exportedPose.qy,
                    exportedPose.qz,
                    exportedPose.qw,
                )
            )
            firstPoseEventWritten = true
        }
    }

    private fun processFrameLegacyMode(
        arFrameTimestampNanos: Long,
        ts: Double,
        sourceFrame: Long,
        camera: com.google.ar.core.Camera,
        intrinsics: CameraIntrinsics?,
    ) {
        if (anchorCreateCooldownFrames > 0) {
            anchorCreateCooldownFrames -= 1
        }

        if (camera.trackingState != TrackingState.TRACKING) {
            if (previousTrackingState == TrackingState.TRACKING) {
                pendingRelocalization = true
                Log.w(tag, "Tracking transitioned TRACKING -> ${camera.trackingState}; will re-anchor on recovery")
            }
            transitionTrackingPipelineState(TrackingPipelineState.INVALID_TRACKING_PAUSED, ts)
            trackingConsecutiveFrames = 0
            recoveringConsecutiveFrames += 1
            previousTrackingState = camera.trackingState
            relocalizationHoldFramesRemaining = 0
            val failureReason = camera.trackingFailureReason
            pauseMediaRecorderIfActive(
                tsUnixSeconds = ts,
                reason = "tracking_invalid:${camera.trackingState.name}:${failureReason.name}"
            )

            val elapsedMs = if (recordingStartElapsedNanos == 0L) 0L
            else (SystemClock.elapsedRealtimeNanos() - recordingStartElapsedNanos) / 1_000_000L

            if (!hadTracking && elapsedMs < 10_000L) {
                onStatus("Waiting for ARCore tracking... (${camera.trackingState})")
                missingPoseCounter.set(0)
                return
            }

            val misses = missingPoseCounter.incrementAndGet()
            if (misses % RECOVERING_STOP_FRAMES == 0) {
                onStatus("Paused: ARCore tracking not active (${camera.trackingState}), waiting for recovery")
                datasetWriter.writeEvent(
                    tsUnixSeconds = ts,
                    event = "tracking_invalid_persistent",
                    details = "misses=$misses,state=${camera.trackingState.name},reason=${failureReason.name}"
                )
            }
            return
        }

        val recoveredFromNonTracking = previousTrackingState != TrackingState.TRACKING
        previousTrackingState = TrackingState.TRACKING

        trackingConsecutiveFrames += 1
        recoveringConsecutiveFrames = 0

        if (!hadTracking && trackingConsecutiveFrames >= STABLE_TRACKING_MIN_FRAMES) {
            transitionTrackingPipelineState(TrackingPipelineState.VALID_RECORDING, ts)
        } else if (hadTracking && recoveredFromNonTracking) {
            transitionTrackingPipelineState(TrackingPipelineState.RESUMED_VALID, ts)
        }

        hadTracking = true
        missingPoseCounter.set(0)

        val pose = camera.pose
        val q = pose.rotationQuaternion // x, y, z, w
        val poseTsRaw = ts
        val poseTs = makePoseTimestampStrictlyIncreasing(poseTsRaw)
        val arKitPose = ArKitConventionMapper.mapPose(
            timestampSeconds = poseTs,
            tx = pose.tx().toDouble(),
            ty = pose.ty().toDouble(),
            tz = pose.tz().toDouble(),
            qx = q[0].toDouble(),
            qy = q[1].toDouble(),
            qz = q[2].toDouble(),
            qw = q[3].toDouble(),
        )

        // Secondary anchor-relative output from the same capture for A/B comparison.
        if (ENABLE_ANCHOR_COMPARISON && !anchorComparisonDisabled && captureAnchor == null &&
            trackingConsecutiveFrames >= ANCHOR_CREATE_MIN_STABLE_FRAMES &&
            anchorCreateCooldownFrames == 0
        ) {
            captureAnchor = try {
                arSession?.createAnchor(pose)
            } catch (_: Throwable) {
                anchorComparisonDisabled = true
                null
            }

            if (captureAnchor != null && !firstAnchorEventWritten) {
                datasetWriter.writeEvent(
                    tsUnixSeconds = ts,
                    event = "capture_anchor_created",
                    details = "sourceFrameIndex=$sourceFrame"
                )
                firstAnchorEventWritten = true
            }

            if (captureAnchor == null) {
                anchorCreateCooldownFrames = ANCHOR_CREATE_COOLDOWN_FRAMES
            }
        }

        val comparisonAnchor = captureAnchor
        var anchorExportedPose: PoseSample? = null
        if (ENABLE_ANCHOR_COMPARISON && comparisonAnchor != null && comparisonAnchor.trackingState == TrackingState.TRACKING) {
            val poseRelativeToAnchor = comparisonAnchor.pose.inverse().compose(pose)
            val aq = poseRelativeToAnchor.rotationQuaternion // x, y, z, w
            anchorExportedPose = ArKitConventionMapper.mapPose(
                timestampSeconds = poseTs,
                tx = poseRelativeToAnchor.tx().toDouble(),
                ty = poseRelativeToAnchor.ty().toDouble(),
                tz = poseRelativeToAnchor.tz().toDouble(),
                qx = aq[0].toDouble(),
                qy = aq[1].toDouble(),
                qz = aq[2].toDouble(),
                qw = aq[3].toDouble(),
            )

            val anchorDelta = computePoseDelta(previousAnchorComparisonPose, anchorExportedPose)
            val anchorHealthy = anchorDelta == null ||
                (anchorDelta.translationMeters <= ANCHOR_MAX_FRAME_DELTA_TRANSLATION_M &&
                    anchorDelta.rotationDegrees <= ANCHOR_MAX_FRAME_DELTA_ROTATION_DEG)

            if (anchorHealthy) {
                anchorHealthyStreak += 1
                anchorUnhealthyStreak = 0
            } else {
                anchorUnhealthyStreak += 1
                anchorHealthyStreak = 0
            }

            if (!anchorHealthy && anchorUnhealthyStreak == ANCHOR_MAX_UNHEALTHY_STREAK) {
                datasetWriter.writeEvent(
                    tsUnixSeconds = anchorExportedPose.timestampSeconds,
                    event = "anchor_health_unstable",
                    details = String.format(
                        Locale.US,
                        "unhealthy_streak=%d,max_delta_m=%.4f,max_delta_deg=%.2f,pose_mode=%s",
                        anchorUnhealthyStreak,
                        ANCHOR_MAX_FRAME_DELTA_TRANSLATION_M,
                        ANCHOR_MAX_FRAME_DELTA_ROTATION_DEG,
                        poseMode.name,
                    )
                )
                try {
                    captureAnchor?.detach()
                } catch (_: Throwable) {
                    // ignore
                }
                captureAnchor = null
                previousAnchorComparisonPose = null
                anchorCreateCooldownFrames = ANCHOR_CREATE_COOLDOWN_FRAMES
                anchorUnhealthyStreak = 0
            } else {
                previousAnchorComparisonPose = anchorExportedPose
            }
        } else if (ENABLE_ANCHOR_COMPARISON && comparisonAnchor != null && comparisonAnchor.trackingState != TrackingState.TRACKING) {
            anchorUnhealthyStreak += 1
            if (anchorUnhealthyStreak >= ANCHOR_MAX_UNHEALTHY_STREAK) {
                try {
                    comparisonAnchor.detach()
                } catch (_: Throwable) {
                    // ignore
                }
                captureAnchor = null
                previousAnchorComparisonPose = null
                anchorCreateCooldownFrames = ANCHOR_CREATE_COOLDOWN_FRAMES
                anchorUnhealthyStreak = 0
                datasetWriter.writeEvent(
                    tsUnixSeconds = ts,
                    event = "anchor_tracking_lost_reset",
                    details = "pose_mode=${poseMode.name}"
                )
            }
        }

        val rawDelta = computePoseDelta(previousRawArKitPose, arKitPose)
        val jumpSuspected = isLikelyWorldReset(rawDelta)
        val severeJumpDetected = isSeverePerFrameJump(rawDelta)
        val recoverySettled = trackingConsecutiveFrames >= STABLE_TRACKING_MIN_FRAMES
        val pendingRelocalizationWithEvidence = pendingRelocalization &&
            (recoveredFromNonTracking || jumpSuspected)
        val immediateRecoveryResetCandidate = pendingRelocalization && recoveredFromNonTracking
        val recoveryResetCandidate = (pendingRelocalizationWithEvidence && recoverySettled) ||
            (recoveredFromNonTracking && recoverySettled)
        val jumpOnlyResetCandidate = jumpSuspected && !pendingRelocalization && !recoveredFromNonTracking
        val resetDetected = ENABLE_CONTINUITY_REANCHOR && (
            immediateRecoveryResetCandidate ||
                recoveryResetCandidate ||
                (ALLOW_JUMP_ONLY_REANCHOR && jumpOnlyResetCandidate)
            )

        if (resetDetected && previousExportedPoseWorld != null) {
            worldReanchorTransform = computeAlignmentTransform(
                fromCurrentRawPose = arKitPose,
                toTargetExportPose = previousExportedPoseWorld!!,
            )
            val resets = relocalizationCounter.incrementAndGet()
            Log.w(
                tag,
                "Pose continuity reset detected (#$resets): sourceFrame=${arFrameTimestampNanos}, " +
                    "trackingRecovered=$recoveredFromNonTracking, pendingRelocalization=$pendingRelocalization, " +
                    "jumpSuspected=$jumpSuspected, rawDelta_m=${rawDelta?.translationMeters}, " +
                    "rawDelta_deg=${rawDelta?.rotationDegrees}"
            )
            datasetWriter.writeEvent(
                tsUnixSeconds = poseTs,
                event = "tracking_relocalized",
                details = String.format(
                    Locale.US,
                    "relocalization_count=%d,recoveredFromNonTracking=%s,pendingRelocalization=%s,pendingRelocalizationWithEvidence=%s,recoverySettled=%s,immediateRecoveryResetCandidate=%s,jumpSuspected=%s,severeJumpDetected=%s,jumpOnlyResetCandidate=%s,allowJumpOnly=%s,pose_mode=%s",
                    resets,
                    recoveredFromNonTracking,
                    pendingRelocalization,
                    pendingRelocalizationWithEvidence,
                    recoverySettled,
                    immediateRecoveryResetCandidate,
                    jumpSuspected,
                    severeJumpDetected,
                    jumpOnlyResetCandidate,
                    ALLOW_JUMP_ONLY_REANCHOR,
                    poseMode.name,
                )
            )
            relocalizationHoldFramesRemaining = RELOCALIZATION_HOLD_FRAMES
            transitionTrackingPipelineState(TrackingPipelineState.RELOCALIZATION_HOLD, poseTs)
            pendingRelocalization = false
        }
        if (recoverySettled) {
            pendingRelocalization = false
            if (relocalizationHoldFramesRemaining <= 0) {
                transitionTrackingPipelineState(TrackingPipelineState.VALID_RECORDING, poseTs)
            }
        }

        val exportedPoseWorld = if (ENABLE_CONTINUITY_REANCHOR) {
            applyTransform(worldReanchorTransform, arKitPose)
        } else {
            arKitPose
        }

        val rejectDiscontinuity = shouldRejectDiscontinuity(rawDelta)
        if (rejectDiscontinuity) {
            pauseMediaRecorderIfActive(ts, "pose_discontinuity_rejected")
            datasetWriter.writeEvent(
                tsUnixSeconds = poseTs,
                event = "pose_rejected_discontinuity",
                details = String.format(
                    Locale.US,
                    "raw_delta_m=%.6f,raw_delta_deg=%.6f,threshold_m=%.3f,threshold_deg=%.1f",
                    rawDelta?.translationMeters ?: 0.0,
                    rawDelta?.rotationDegrees ?: 0.0,
                    DISCONTINUITY_REJECT_TRANSLATION_M,
                    DISCONTINUITY_REJECT_ROTATION_DEG,
                )
            )
            previousRawArKitPose = arKitPose
            resetTrajectoryFilterState()
            return
        }

        val exportContinuityDelta = computePoseDelta(previousExportedPoseWorld, exportedPoseWorld)
        val rejectExportContinuity = shouldRejectAcceptedContinuity(exportContinuityDelta)
        if (rejectExportContinuity) {
            pauseMediaRecorderIfActive(ts, "pose_export_continuity_rejected")
            if (previousExportedPoseWorld != null && ENABLE_CONTINUITY_REANCHOR) {
                worldReanchorTransform = computeAlignmentTransform(
                    fromCurrentRawPose = arKitPose,
                    toTargetExportPose = previousExportedPoseWorld!!,
                )
            }
            relocalizationHoldFramesRemaining = RELOCALIZATION_HOLD_FRAMES
            transitionTrackingPipelineState(TrackingPipelineState.RELOCALIZATION_HOLD, poseTs)
            datasetWriter.writeEvent(
                tsUnixSeconds = poseTs,
                event = "pose_rejected_export_continuity",
                details = String.format(
                    Locale.US,
                    "export_delta_m=%.6f,export_delta_deg=%.6f,max_m=%.3f,max_deg=%.1f",
                    exportContinuityDelta?.translationMeters ?: 0.0,
                    exportContinuityDelta?.rotationDegrees ?: 0.0,
                    ACCEPTED_CONTINUITY_MAX_TRANSLATION_M,
                    ACCEPTED_CONTINUITY_MAX_ROTATION_DEG,
                )
            )
            previousRawArKitPose = arKitPose
            resetTrajectoryFilterState()
            return
        }

        if (relocalizationHoldFramesRemaining > 0) {
            relocalizationHoldFramesRemaining -= 1
            pauseMediaRecorderIfActive(ts, "relocalization_hold")
            transitionTrackingPipelineState(TrackingPipelineState.RELOCALIZATION_HOLD, ts)
            datasetWriter.writeEvent(
                tsUnixSeconds = poseTs,
                event = "pose_hold_relocalization",
                details = "remaining_frames=$relocalizationHoldFramesRemaining,pose_mode=${poseMode.name}"
            )
            previousRawArKitPose = arKitPose
            resetTrajectoryFilterState()
            return
        }

        resumeMediaRecorderIfPaused(ts)
        transitionTrackingPipelineState(TrackingPipelineState.VALID_RECORDING, ts)

        val exportRelativePose = if (ENABLE_FIRST_FRAME_RELATIVE_EXPORT) {
            computeFirstFrameRelativePose(exportedPoseWorld)
        } else {
            exportedPoseWorld
        }

        val exportedPose = if (ENABLE_KALMAN_FILTER) {
            filterPoseDeterministic(previousSmoothedPoseRelative, exportRelativePose)
        } else {
            exportRelativePose
        }

        val continuityClampedPose = clampOutputStepToContinuity(
            previous = previousExportedPoseRelative,
            candidate = exportedPose,
        )

        if (continuityClampedPose !== exportedPose) {
            datasetWriter.writeEvent(
                tsUnixSeconds = continuityClampedPose.timestampSeconds,
                event = "pose_output_clamped",
                details = String.format(
                    Locale.US,
                    "max_step_m=%.3f,max_step_deg=%.1f",
                    ACCEPTED_CONTINUITY_MAX_TRANSLATION_M,
                    ACCEPTED_CONTINUITY_MAX_ROTATION_DEG,
                )
            )
        }

        datasetWriter.writeRawPose(
            exportRelativePose.timestampSeconds,
            exportRelativePose.tx,
            exportRelativePose.ty,
            exportRelativePose.tz,
            exportRelativePose.qx,
            exportRelativePose.qy,
            exportRelativePose.qz,
            exportRelativePose.qw,
        )

        updateAndLogFilterLagDiagnostics(
            tsUnixSeconds = continuityClampedPose.timestampSeconds,
            rawPose = exportRelativePose,
            filteredPose = continuityClampedPose,
        )

        val exportedDelta = computePoseDelta(previousExportedPoseRelative, continuityClampedPose)
        val exportedTranslationBreakdown = computeTranslationDeltaBreakdown(previousExportedPoseRelative, continuityClampedPose)

        val frame = frameCounter.getAndIncrement()
        datasetWriter.writeFrame(ts, frame, intrinsics)

        if (!firstFrameEventWritten) {
            datasetWriter.writeEvent(
                tsUnixSeconds = ts,
                event = "first_frame_seen",
                details = "frameIndex=$frame,sourceFrameIndex=$sourceFrame"
            )
            firstFrameEventWritten = true
        }

        logPoseInstrumentation(
            sourceFrameIndex = sourceFrame,
            sourceFrameTimestampNanos = arFrameTimestampNanos,
            trackingState = camera.trackingState,
            relocalizationOrReset = resetDetected,
            rawDelta = rawDelta,
            exportedDelta = exportedDelta,
            exportedTranslationBreakdown = exportedTranslationBreakdown,
        )

        val continuityClampedPoseFixed = applyArposesUpAxisHalfTurnFix(continuityClampedPose)
        datasetWriter.writePose(
            continuityClampedPose.timestampSeconds,
            continuityClampedPose.tx,
            continuityClampedPose.ty,
            continuityClampedPose.tz,
            continuityClampedPoseFixed.qx,
            continuityClampedPoseFixed.qy,
            continuityClampedPoseFixed.qz,
            continuityClampedPoseFixed.qw,
        )

        if (!firstPoseEventWritten) {
            datasetWriter.writeEvent(
                tsUnixSeconds = continuityClampedPose.timestampSeconds,
                event = "first_pose_written",
                details = String.format(
                    Locale.US,
                    "frameIndex=%d,tx=%.6f,ty=%.6f,tz=%.6f,qx=%.6f,qy=%.6f,qz=%.6f,qw=%.6f",
                    frame,
                    continuityClampedPose.tx,
                    continuityClampedPose.ty,
                    continuityClampedPose.tz,
                    continuityClampedPose.qx,
                    continuityClampedPose.qy,
                    continuityClampedPose.qz,
                    continuityClampedPose.qw,
                )
            )
            firstPoseEventWritten = true
        }

        previousRawArKitPose = arKitPose
        previousExportedPoseWorld = exportedPoseWorld
        previousExportedPoseRelative = continuityClampedPose
        if (ENABLE_KALMAN_FILTER) {
            previousSmoothedPoseRelative = continuityClampedPose
        }
    }

    private fun exportedPoseTsForTransition(poseTs: Double): Double = poseTs

    private fun applyArposesUpAxisHalfTurnFix(pose: PoseSample): ExportPoseComponents {
        // Required by ARposes export contract:
        // camera-to-world rotation post-multiply by Ry(pi), i.e. q' = q ⊗ q_fix,
        // where q_fix (wxyz) = (0, 0, 1, 0).
        val q = normalizeQuaternionWxyz(doubleArrayOf(pose.qw, pose.qx, pose.qy, pose.qz))
        val qFix = doubleArrayOf(0.0, 0.0, 1.0, 0.0)
        val qOut = normalizeQuaternionWxyz(multiplyQuaternionWxyzRaw(q, qFix))
        return ExportPoseComponents(
            qw = qOut[0],
            qx = qOut[1],
            qy = qOut[2],
            qz = qOut[3],
        )
    }

    private fun shouldRejectDiscontinuity(rawDelta: PoseDelta?): Boolean {
        if (rawDelta == null) return false
        return rawDelta.translationMeters >= DISCONTINUITY_REJECT_TRANSLATION_M ||
            rawDelta.rotationDegrees >= DISCONTINUITY_REJECT_ROTATION_DEG
    }

    private fun shouldRejectAcceptedContinuity(exportedDelta: PoseDelta?): Boolean {
        if (exportedDelta == null) return false
        return exportedDelta.translationMeters >= ACCEPTED_CONTINUITY_MAX_TRANSLATION_M ||
            exportedDelta.rotationDegrees >= ACCEPTED_CONTINUITY_MAX_ROTATION_DEG
    }


    private fun computeFirstFrameRelativePose(currentWorldPose: PoseSample): PoseSample {
        if (firstReferenceWorldPose == null) {
            firstReferenceWorldPose = currentWorldPose
            datasetWriter.writeEvent(
                tsUnixSeconds = currentWorldPose.timestampSeconds,
                event = "first_frame_reference_set",
                details = String.format(
                    Locale.US,
                    "mode=offset_only,tx=%.6f,ty=%.6f,tz=%.6f,qx=%.6f,qy=%.6f,qz=%.6f,qw=%.6f",
                    currentWorldPose.tx,
                    currentWorldPose.ty,
                    currentWorldPose.tz,
                    currentWorldPose.qx,
                    currentWorldPose.qy,
                    currentWorldPose.qz,
                    currentWorldPose.qw,
                )
            )
        }

        val ref = firstReferenceWorldPose ?: return currentWorldPose

        // Offset-only translation normalization (no axis re-basing / no rotation of translation).
        val tx = currentWorldPose.tx - ref.tx
        val ty = currentWorldPose.ty - ref.ty
        val tz = currentWorldPose.tz - ref.tz

        // Relative orientation to first frame so the first sample becomes identity before
        // ARposes up-axis half-turn fix (which then yields the expected qFix convention).
        val qRef = normalizeQuaternionWxyz(doubleArrayOf(ref.qw, ref.qx, ref.qy, ref.qz))
        val qCurrent = normalizeQuaternionWxyz(
            doubleArrayOf(currentWorldPose.qw, currentWorldPose.qx, currentWorldPose.qy, currentWorldPose.qz)
        )
        val qRel = normalizeQuaternionWxyz(
            multiplyQuaternionWxyz(conjugateQuaternionWxyz(qRef), qCurrent)
        )

        return PoseSample(
            timestampSeconds = currentWorldPose.timestampSeconds,
            tx = tx,
            ty = ty,
            tz = tz,
            qw = qRel[0],
            qx = qRel[1],
            qy = qRel[2],
            qz = qRel[3],
        )
    }

    private fun filterPoseDeterministic(previous: PoseSample?, current: PoseSample): PoseSample {
        if (previous == null) return current

        val tx = kalmanTx.update(current.tx)
        val ty = kalmanTy.update(current.ty)
        val tz = kalmanTz.update(current.tz)

        val prevQ = normalizeQuaternionWxyz(doubleArrayOf(previous.qw, previous.qx, previous.qy, previous.qz))
        val currQ = normalizeQuaternionWxyz(doubleArrayOf(current.qw, current.qx, current.qy, current.qz))
        val smoothedQ = slerpQuaternionWxyz(prevQ, currQ, ROTATION_SMOOTHING_ALPHA)
        return PoseSample(
            timestampSeconds = current.timestampSeconds,
            tx = tx,
            ty = ty,
            tz = tz,
            qw = smoothedQ[0],
            qx = smoothedQ[1],
            qy = smoothedQ[2],
            qz = smoothedQ[3],
        )
    }

    private fun resetTrajectoryFilterState() {
        previousSmoothedPoseRelative = null
        kalmanTx.reset()
        kalmanTy.reset()
        kalmanTz.reset()
        lagEstimator.reset()
        previousRawRelativePoseForLag = null
        previousFilteredPoseForLag = null
        previousLagSampleTimestamp = null
        lastLoggedLagFrames = Int.MIN_VALUE
    }

    private fun updateAndLogFilterLagDiagnostics(
        tsUnixSeconds: Double,
        rawPose: PoseSample,
        filteredPose: PoseSample,
    ) {
        val prevRaw = previousRawRelativePoseForLag
        val prevFiltered = previousFilteredPoseForLag
        val prevTs = previousLagSampleTimestamp

        if (prevRaw != null && prevFiltered != null && prevTs != null) {
            val rawStep = positionStepMeters(prevRaw, rawPose)
            val filteredStep = positionStepMeters(prevFiltered, filteredPose)
            val dt = (tsUnixSeconds - prevTs).coerceAtLeast(0.0)
            lagEstimator.addSample(rawStep, filteredStep, dt)
            val lag = lagEstimator.estimate()
            if (lag != null) {
                datasetWriter.writeFilterDiagnostic(
                    ts = tsUnixSeconds,
                    lagFrames = lag.lagFrames,
                    lagSeconds = lag.lagSeconds,
                    corr = lag.correlation,
                )
                if (lag.lagFrames != lastLoggedLagFrames) {
                    datasetWriter.writeEvent(
                        tsUnixSeconds = tsUnixSeconds,
                        event = "filter_lag_estimate",
                        details = String.format(
                            Locale.US,
                            "lag_frames=%d,lag_seconds=%.6f,corr=%.4f",
                            lag.lagFrames,
                            lag.lagSeconds,
                            lag.correlation,
                        )
                    )
                    lastLoggedLagFrames = lag.lagFrames
                }
            }
        }

        previousRawRelativePoseForLag = rawPose
        previousFilteredPoseForLag = filteredPose
        previousLagSampleTimestamp = tsUnixSeconds
    }

    private fun positionStepMeters(a: PoseSample, b: PoseSample): Double {
        val dx = b.tx - a.tx
        val dy = b.ty - a.ty
        val dz = b.tz - a.tz
        return sqrt(dx * dx + dy * dy + dz * dz)
    }

    private fun clampOutputStepToContinuity(previous: PoseSample?, candidate: PoseSample): PoseSample {
        if (previous == null) return candidate
        val delta = computePoseDelta(previous, candidate) ?: return candidate

        val tTrans = if (delta.translationMeters > ACCEPTED_CONTINUITY_MAX_TRANSLATION_M) {
            ACCEPTED_CONTINUITY_MAX_TRANSLATION_M / delta.translationMeters
        } else {
            1.0
        }
        val tRot = if (delta.rotationDegrees > ACCEPTED_CONTINUITY_MAX_ROTATION_DEG) {
            ACCEPTED_CONTINUITY_MAX_ROTATION_DEG / delta.rotationDegrees
        } else {
            1.0
        }

        val t = min(1.0, min(tTrans, tRot))
        if (t >= 1.0) return candidate

        val tx = previous.tx + (candidate.tx - previous.tx) * t
        val ty = previous.ty + (candidate.ty - previous.ty) * t
        val tz = previous.tz + (candidate.tz - previous.tz) * t

        val prevQ = normalizeQuaternionWxyz(doubleArrayOf(previous.qw, previous.qx, previous.qy, previous.qz))
        val candQ = normalizeQuaternionWxyz(doubleArrayOf(candidate.qw, candidate.qx, candidate.qy, candidate.qz))
        val q = slerpQuaternionWxyz(prevQ, candQ, t)

        return PoseSample(
            timestampSeconds = candidate.timestampSeconds,
            tx = tx,
            ty = ty,
            tz = tz,
            qw = q[0],
            qx = q[1],
            qy = q[2],
            qz = q[3],
        )
    }

    private fun slerpQuaternionWxyz(aIn: DoubleArray, bIn: DoubleArray, t: Double): DoubleArray {
        var a = normalizeQuaternionWxyz(aIn)
        var b = normalizeQuaternionWxyz(bIn)
        var dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]

        if (dot < 0.0) {
            b = doubleArrayOf(-b[0], -b[1], -b[2], -b[3])
            dot = -dot
        }

        if (dot > 0.9995) {
            val out = doubleArrayOf(
                a[0] + t * (b[0] - a[0]),
                a[1] + t * (b[1] - a[1]),
                a[2] + t * (b[2] - a[2]),
                a[3] + t * (b[3] - a[3]),
            )
            return normalizeQuaternionWxyz(out)
        }

        val theta0 = acos(dot)
        val theta = theta0 * t
        val sinTheta = kotlin.math.sin(theta)
        val sinTheta0 = kotlin.math.sin(theta0)
        val s0 = kotlin.math.cos(theta) - dot * sinTheta / sinTheta0
        val s1 = sinTheta / sinTheta0

        return normalizeQuaternionWxyz(
            doubleArrayOf(
                s0 * a[0] + s1 * b[0],
                s0 * a[1] + s1 * b[1],
                s0 * a[2] + s1 * b[2],
                s0 * a[3] + s1 * b[3],
            )
        )
    }

    private fun pauseMediaRecorderIfActive(tsUnixSeconds: Double, reason: String) {
        if (mediaRecorderPaused || !started.get()) return
        try {
            mediaRecorder?.pause()
            mediaRecorderPaused = true
            datasetWriter.writeEvent(
                tsUnixSeconds = tsUnixSeconds,
                event = "capture_paused",
                details = reason,
            )
        } catch (_: Throwable) {
            // Best effort only; frame/pose gating still enforces artifact consistency.
        }
    }

    private fun resumeMediaRecorderIfPaused(tsUnixSeconds: Double) {
        if (!mediaRecorderPaused || !started.get()) return
        try {
            mediaRecorder?.resume()
            mediaRecorderPaused = false
            datasetWriter.writeEvent(
                tsUnixSeconds = tsUnixSeconds,
                event = "capture_resumed",
                details = "tracking_valid",
            )
        } catch (_: Throwable) {
            // Best effort only; frame/pose gating still enforces artifact consistency.
        }
    }

    private fun transitionTrackingPipelineState(newState: TrackingPipelineState, tsUnixSeconds: Double) {
        if (trackingPipelineState == newState) return
        val oldState = trackingPipelineState
        trackingPipelineState = newState
        datasetWriter.writeEvent(
            tsUnixSeconds = tsUnixSeconds,
            event = "tracking_fsm_transition",
            details = "from=${oldState.name},to=${newState.name},pose_mode=${poseMode.name}"
        )
        Log.i(tag, "tracking_fsm_transition: ${oldState.name} -> ${newState.name}")
    }

    private fun mapIntrinsicsToOutputResolution(
        intrinsics: CameraIntrinsics,
        srcWidth: Int,
        srcHeight: Int,
        dstWidth: Int,
        dstHeight: Int,
    ): CameraIntrinsics {
        if (srcWidth <= 0 || srcHeight <= 0 || dstWidth <= 0 || dstHeight <= 0) {
            return intrinsics
        }

        val srcAspect = srcWidth.toDouble() / srcHeight.toDouble()
        val dstAspect = dstWidth.toDouble() / dstHeight.toDouble()
        val eps = 1e-9

        val cropX: Double
        val cropY: Double
        val cropWidth: Double
        val cropHeight: Double

        if (kotlin.math.abs(srcAspect - dstAspect) <= eps) {
            cropX = 0.0
            cropY = 0.0
            cropWidth = srcWidth.toDouble()
            cropHeight = srcHeight.toDouble()
        } else if (srcAspect > dstAspect) {
            cropHeight = srcHeight.toDouble()
            cropWidth = cropHeight * dstAspect
            cropX = (srcWidth - cropWidth) * 0.5
            cropY = 0.0
        } else {
            cropWidth = srcWidth.toDouble()
            cropHeight = cropWidth / dstAspect
            cropX = 0.0
            cropY = (srcHeight - cropHeight) * 0.5
        }

        val sx = dstWidth / cropWidth
        val sy = dstHeight / cropHeight

        val mapped = CameraIntrinsics(
            fx = intrinsics.fx * sx,
            fy = intrinsics.fy * sy,
            cx = (intrinsics.cx - cropX) * sx,
            cy = (intrinsics.cy - cropY) * sy,
        )

        return mapped
    }

    private fun makePoseTimestampStrictlyIncreasing(rawTs: Double): Double {
        if (rawTs > lastPoseTimestampSeconds) {
            lastPoseTimestampSeconds = rawTs
            return rawTs
        }

        val adjusted = lastPoseTimestampSeconds + 1e-6
        lastPoseTimestampSeconds = adjusted
        val adjustments = adjustedPoseTimestampCounter.incrementAndGet()
        if (adjustments <= 5 || adjustments % 100 == 0) {
            Log.w(
                tag,
                "Adjusted non-increasing pose timestamp raw=$rawTs adjusted=$adjusted totalAdjustments=$adjustments"
            )
        }
        return adjusted
    }

    private fun openSharedCamera() {
        val session = arSession ?: throw IllegalStateException("ARCore session not initialized")
        val shared = sharedCamera ?: throw IllegalStateException("SharedCamera not initialized")

        if (!hasCameraPermission()) {
            onStatus("Start failed: CAMERA permission missing")
            stopInternal("camera permission missing")
            return
        }

        val cameraId = session.cameraConfig.cameraId
        val cameraManager = context.getSystemService(Context.CAMERA_SERVICE) as CameraManager

        val appSurfaces = mutableListOf<Surface>()
        previewSurface?.let { appSurfaces += it }
        mediaRecorder?.surface?.let { appSurfaces += it }
        imageReader?.surface?.let { appSurfaces += it }
        shared.setAppSurfaces(cameraId, appSurfaces)

        val wrappedDeviceCallback = shared.createARDeviceStateCallback(
            object : CameraDevice.StateCallback() {
                override fun onOpened(device: CameraDevice) {
                    cameraDevice = device
                    Log.i(tag, "CameraDevice opened: ${device.id}")
                    createCaptureSession(device)
                }

                override fun onDisconnected(device: CameraDevice) {
                    onStatus("Stopped: camera disconnected")
                    Log.e(tag, "CameraDevice disconnected: ${device.id}")
                    stopInternal("camera disconnected")
                }

                override fun onError(device: CameraDevice, error: Int) {
                    onStatus("Stopped: camera device error $error")
                    Log.e(tag, "CameraDevice error $error on id=${device.id}")
                    stopInternal("camera error $error")
                }
            },
            cameraHandler
        )

        try {
            cameraManager.openCamera(cameraId, wrappedDeviceCallback, cameraHandler)
        } catch (se: SecurityException) {
            Log.e(tag, "openSharedCamera: CAMERA permission denied", se)
            onStatus("Start failed: CAMERA permission denied")
            stopInternal("camera permission denied")
        }
    }

    private fun hasCameraPermission(): Boolean {
        return ContextCompat.checkSelfPermission(context, Manifest.permission.CAMERA) ==
            PackageManager.PERMISSION_GRANTED
    }

    private fun createCaptureSession(device: CameraDevice) {
        val shared = sharedCamera ?: throw IllegalStateException("SharedCamera not initialized")

        val surfaces = mutableListOf<Surface>()
        previewSurface?.let { surfaces += it }
        mediaRecorder?.surface?.let { surfaces += it }
        imageReader?.surface?.let { surfaces += it }
        surfaces += shared.getArCoreSurfaces()

        val wrappedSessionCallback = shared.createARSessionStateCallback(
            object : CameraCaptureSession.StateCallback() {
                override fun onConfigured(session: CameraCaptureSession) {
                    captureSession = session
                    Log.i(tag, "CameraCaptureSession configured")
                    startRepeatingRequest(session)
                }

                override fun onConfigureFailed(session: CameraCaptureSession) {
                    onStatus("Stopped: camera capture session configure failed")
                    Log.e(tag, "CameraCaptureSession configure failed")
                    stopInternal("configure failed")
                }
            },
            cameraHandler
        )

        device.createCaptureSession(surfaces, wrappedSessionCallback, cameraHandler)
    }

    private fun startRepeatingRequest(session: CameraCaptureSession) {
        val device = cameraDevice ?: throw IllegalStateException("CameraDevice not ready")

        val requestBuilder = device.createCaptureRequest(CameraDevice.TEMPLATE_RECORD)

        previewSurface?.let { requestBuilder.addTarget(it) }
        mediaRecorder?.surface?.let { requestBuilder.addTarget(it) }
        imageReader?.surface?.let { requestBuilder.addTarget(it) }
        sharedCamera?.getArCoreSurfaces()?.forEach { requestBuilder.addTarget(it) }

        captureRequest = requestBuilder.build()

        try {
            arSession?.resume()
            mediaRecorder?.start()
            session.setRepeatingRequest(captureRequest!!, null, cameraHandler)

            started.set(true)
            recordingStartElapsedNanos = SystemClock.elapsedRealtimeNanos()
            hadTracking = false
            sourceFrameCounter.set(0)
            frameCounter.set(0)
            lastPoseTimestampSeconds = Double.NEGATIVE_INFINITY
            adjustedPoseTimestampCounter.set(0)
            relocalizationCounter.set(0)
            missingPoseCounter.set(0)
            previousTrackingState = null
            pendingRelocalization = false
            previousRawArKitPose = null
            previousExportedPoseWorld = null
            previousExportedPoseRelative = null
            firstReferenceWorldPose = null
            resetTrajectoryFilterState()
            worldReanchorTransform = null
            captureAnchor = null
            anchorComparisonDisabled = false
            if (!ENABLE_ANCHOR_COMPARISON) {
                anchorComparisonDisabled = true
            }
            trackingPipelineState = TrackingPipelineState.WARMUP
            trackingConsecutiveFrames = 0
            recoveringConsecutiveFrames = 0
            relocalizationHoldFramesRemaining = 0
            mediaRecorderPaused = false
            previousAnchorComparisonPose = null
            anchorHealthyStreak = 0
            anchorUnhealthyStreak = 0
            anchorCreateCooldownFrames = 0
            firstAnchorEventWritten = false
            firstFrameEventWritten = false
            firstPoseEventWritten = false
            lastIntrinsicsFingerprint = null
            datasetWriter.writeEvent(
                tsUnixSeconds = System.currentTimeMillis() / 1000.0,
                event = "engine_started",
                details = "shared_camera=true,pose_mode=${poseMode.name},first_frame_relative=$ENABLE_FIRST_FRAME_RELATIVE_EXPORT,first_frame_relative_mode=offset_only,filter=kalman,rotation_alpha=$ROTATION_SMOOTHING_ALPHA,timestamp_policy=frame_time"
            )
            onStarted()
            onTrackingState("STARTED")
            onStatus("Recording (ARCore SharedCamera)")
            Log.i(tag, "Recording started")
        } catch (e: Throwable) {
            onStatus("Start failed: ${e.message ?: e::class.java.simpleName}")
            onTrackingState("ERROR")
            Log.e(tag, "startRepeatingRequest(): failed", e)
            stopInternal("repeat request failed")
        }
    }

    private fun stopInternal(reason: String?) {
        if (!stopping.compareAndSet(false, true)) return
        Log.i(tag, "stopInternal(): reason=$reason")

        try {
            if (started.get()) {
                try {
                    mediaRecorder?.stop()
                } catch (_: Throwable) {
                    // stop may throw if recorder wasn't fully started.
                }
            }
            mediaRecorder?.reset()
            mediaRecorder?.release()
        } catch (_: Throwable) {
            // ignore
        } finally {
            mediaRecorder = null
        }

        try {
            captureSession?.stopRepeating()
        } catch (_: Throwable) {
            // ignore
        }

        try {
            captureSession?.abortCaptures()
        } catch (_: Throwable) {
            // ignore
        }

        try {
            captureSession?.close()
        } catch (_: Throwable) {
            // ignore
        } finally {
            captureSession = null
        }

        try {
            cameraDevice?.close()
        } catch (_: Throwable) {
            // ignore
        } finally {
            cameraDevice = null
        }

        try {
            imageReader?.close()
        } catch (_: Throwable) {
            // ignore
        } finally {
            imageReader = null
        }

        try {
            captureAnchor?.detach()
        } catch (_: Throwable) {
            // ignore
        } finally {
            captureAnchor = null
        }

        try {
            arSession?.pause()
            arSession?.close()
        } catch (_: Throwable) {
            // ignore
        } finally {
            arSession = null
            sharedCamera = null
        }

        releaseOffscreenGlContext()

        try {
            cameraThread?.quitSafely()
        } catch (_: Throwable) {
            // ignore
        } finally {
            cameraThread = null
            cameraHandler = null
        }

        onTrackingState("STOPPED")
        val adjustedCount = adjustedPoseTimestampCounter.get()
        if (adjustedCount > 0) {
            Log.i(tag, "stopInternal(): adjusted non-increasing pose timestamps count=$adjustedCount")
        }
        val resetCount = relocalizationCounter.get()
        if (resetCount > 0) {
            Log.i(tag, "stopInternal(): pose continuity resets/re-anchors count=$resetCount")
        }
        onStopped(reason)
    }

    private fun logPoseInstrumentation(
        sourceFrameIndex: Long,
        sourceFrameTimestampNanos: Long,
        trackingState: TrackingState,
        relocalizationOrReset: Boolean,
        rawDelta: PoseDelta?,
        exportedDelta: PoseDelta?,
        exportedTranslationBreakdown: TranslationDeltaBreakdown?,
    ) {
        if (!POSE_INSTRUMENTATION_LOG_EVERY_FRAME && !relocalizationOrReset) return

        val rawDm = rawDelta?.translationMeters ?: 0.0
        val rawDdeg = rawDelta?.rotationDegrees ?: 0.0
        val expDm = exportedDelta?.translationMeters ?: 0.0
        val expDdeg = exportedDelta?.rotationDegrees ?: 0.0

        val worldDx = exportedTranslationBreakdown?.worldDx ?: 0.0
        val worldDy = exportedTranslationBreakdown?.worldDy ?: 0.0
        val worldDz = exportedTranslationBreakdown?.worldDz ?: 0.0
        val cameraDx = exportedTranslationBreakdown?.cameraDx ?: 0.0
        val cameraDy = exportedTranslationBreakdown?.cameraDy ?: 0.0
        val cameraDz = exportedTranslationBreakdown?.cameraDz ?: 0.0

        Log.i(
            tag,
            "pose_instrumentation frameIdx=$sourceFrameIndex srcFrameNs=$sourceFrameTimestampNanos " +
                "tracking=$trackingState reset=$relocalizationOrReset chain='$TRANSFORM_CHAIN_ID' " +
                "rawDelta_m=$rawDm rawDelta_deg=$rawDdeg exportedDelta_m=$expDm exportedDelta_deg=$expDdeg " +
                "exportedWorldDelta_xyz=($worldDx,$worldDy,$worldDz) " +
                "exportedCameraLocalDelta_xyz=($cameraDx,$cameraDy,$cameraDz)"
        )
    }

    private fun logIntrinsicsConsistency(
        tsUnixSeconds: Double,
        sourceFrameIndex: Long,
        snapshot: IntrinsicsSnapshot,
    ) {
        val mapped = snapshot.mapped
        val finite = mapped.fx.isFinite() && mapped.fy.isFinite() && mapped.cx.isFinite() && mapped.cy.isFinite()
        val positiveFocal = mapped.fx > 0.0 && mapped.fy > 0.0
        val principalInBounds =
            mapped.cx >= 0.0 && mapped.cx <= resolution.width.toDouble() &&
                mapped.cy >= 0.0 && mapped.cy <= resolution.height.toDouble()
        val srcDimsValid = snapshot.srcWidth > 0 && snapshot.srcHeight > 0

        val status = if (finite && positiveFocal && principalInBounds && srcDimsValid) "ok" else "invalid"
        val fingerprint = String.format(
            Locale.US,
            "src=%dx%d,dst=%dx%d,raw=[%.2f,%.2f,%.2f,%.2f],mapped=[%.2f,%.2f,%.2f,%.2f],status=%s",
            snapshot.srcWidth,
            snapshot.srcHeight,
            resolution.width,
            resolution.height,
            snapshot.raw.fx,
            snapshot.raw.fy,
            snapshot.raw.cx,
            snapshot.raw.cy,
            mapped.fx,
            mapped.fy,
            mapped.cx,
            mapped.cy,
            status,
        )

        val changed = lastIntrinsicsFingerprint != fingerprint
        if (!changed && status == "ok") {
            return
        }

        lastIntrinsicsFingerprint = fingerprint
        val message = "intrinsics_consistency frameIdx=$sourceFrameIndex ts=$tsUnixSeconds $fingerprint"

        if (status == "ok") {
            Log.i(tag, message)
            datasetWriter.writeEvent(
                tsUnixSeconds = tsUnixSeconds,
                event = "intrinsics_consistency",
                details = message,
            )
        } else {
            Log.e(tag, message)
            datasetWriter.writeEvent(
                tsUnixSeconds = tsUnixSeconds,
                event = "intrinsics_consistency_invalid",
                details = message,
            )
        }
    }

    private fun computeTranslationDeltaBreakdown(
        previous: PoseSample?,
        current: PoseSample,
    ): TranslationDeltaBreakdown? {
        if (previous == null) return null

        val dx = current.tx - previous.tx
        val dy = current.ty - previous.ty
        val dz = current.tz - previous.tz

        val qPrevWxyz = normalizeQuaternionWxyz(
            doubleArrayOf(previous.qw, previous.qx, previous.qy, previous.qz)
        )
        val qPrevInv = conjugateQuaternionWxyz(qPrevWxyz)
        val dCamera = rotateVectorByQuaternionWxyz(qPrevInv, doubleArrayOf(dx, dy, dz))

        return TranslationDeltaBreakdown(
            worldDx = dx,
            worldDy = dy,
            worldDz = dz,
            cameraDx = dCamera[0],
            cameraDy = dCamera[1],
            cameraDz = dCamera[2],
        )
    }

    private fun computePoseDelta(previous: PoseSample?, current: PoseSample): PoseDelta? {
        if (previous == null) return null

        val dx = current.tx - previous.tx
        val dy = current.ty - previous.ty
        val dz = current.tz - previous.tz
        val translation = sqrt(dx * dx + dy * dy + dz * dz)

        val relQ = multiplyQuaternionWxyz(
            conjugateQuaternionWxyz(doubleArrayOf(previous.qw, previous.qx, previous.qy, previous.qz)),
            doubleArrayOf(current.qw, current.qx, current.qy, current.qz),
        )
        val relQNorm = normalizeQuaternionWxyz(relQ)
        val angleRad = 2.0 * acos(min(1.0, max(-1.0, abs(relQNorm[0]))))

        return PoseDelta(
            translationMeters = translation,
            rotationDegrees = Math.toDegrees(angleRad),
        )
    }

    private fun isLikelyWorldReset(rawDelta: PoseDelta?): Boolean {
        if (rawDelta == null) return false
        return rawDelta.translationMeters >= HARD_TRANSLATION_RESET_M ||
            rawDelta.rotationDegrees >= HARD_ROTATION_RESET_DEG ||
            (rawDelta.translationMeters >= COMBINED_TRANSLATION_RESET_M &&
                rawDelta.rotationDegrees >= COMBINED_ROTATION_RESET_DEG)
    }

    private fun isSeverePerFrameJump(rawDelta: PoseDelta?): Boolean {
        if (rawDelta == null) return false
        return rawDelta.translationMeters >= PHYSICALLY_IMPLAUSIBLE_TRANSLATION_JUMP_M ||
            rawDelta.rotationDegrees >= PHYSICALLY_IMPLAUSIBLE_ROTATION_JUMP_DEG
    }

    private fun computeAlignmentTransform(
        fromCurrentRawPose: PoseSample,
        toTargetExportPose: PoseSample,
    ): RigidTransform {
        val tTarget = poseToTransform(toTargetExportPose)
        val tRawCurrent = poseToTransform(fromCurrentRawPose)
        return composeTransforms(tTarget, invertTransform(tRawCurrent))
    }

    private fun applyTransform(transform: RigidTransform?, pose: PoseSample): PoseSample {
        if (transform == null) return pose
        val out = composeTransforms(transform, poseToTransform(pose))
        return transformToPoseSample(out, timestampSeconds = pose.timestampSeconds)
    }

    private fun poseToTransform(pose: PoseSample): RigidTransform {
        return RigidTransform(
            tx = pose.tx,
            ty = pose.ty,
            tz = pose.tz,
            qw = pose.qw,
            qx = pose.qx,
            qy = pose.qy,
            qz = pose.qz,
        )
    }

    private fun transformToPoseSample(transform: RigidTransform, timestampSeconds: Double): PoseSample {
        val q = normalizeQuaternionWxyz(doubleArrayOf(transform.qw, transform.qx, transform.qy, transform.qz))
        return PoseSample(
            timestampSeconds = timestampSeconds,
            tx = transform.tx,
            ty = transform.ty,
            tz = transform.tz,
            qw = q[0],
            qx = q[1],
            qy = q[2],
            qz = q[3],
        )
    }

    private fun composeTransforms(a: RigidTransform, b: RigidTransform): RigidTransform {
        val qa = normalizeQuaternionWxyz(doubleArrayOf(a.qw, a.qx, a.qy, a.qz))
        val qb = normalizeQuaternionWxyz(doubleArrayOf(b.qw, b.qx, b.qy, b.qz))
        val qOut = multiplyQuaternionWxyz(qa, qb)

        val rb = rotateVectorByQuaternionWxyz(qa, doubleArrayOf(b.tx, b.ty, b.tz))
        return RigidTransform(
            tx = rb[0] + a.tx,
            ty = rb[1] + a.ty,
            tz = rb[2] + a.tz,
            qw = qOut[0],
            qx = qOut[1],
            qy = qOut[2],
            qz = qOut[3],
        )
    }

    private fun invertTransform(t: RigidTransform): RigidTransform {
        val q = normalizeQuaternionWxyz(doubleArrayOf(t.qw, t.qx, t.qy, t.qz))
        val qInv = conjugateQuaternionWxyz(q)
        val negT = doubleArrayOf(-t.tx, -t.ty, -t.tz)
        val tInv = rotateVectorByQuaternionWxyz(qInv, negT)
        return RigidTransform(
            tx = tInv[0],
            ty = tInv[1],
            tz = tInv[2],
            qw = qInv[0],
            qx = qInv[1],
            qy = qInv[2],
            qz = qInv[3],
        )
    }

    private fun rotateVectorByQuaternionWxyz(qWxyz: DoubleArray, v: DoubleArray): DoubleArray {
        val q = normalizeQuaternionWxyz(qWxyz)
        val qv = doubleArrayOf(0.0, v[0], v[1], v[2])
        val qConj = conjugateQuaternionWxyz(q)
        // IMPORTANT: vector rotation must preserve vector magnitude.
        // Do NOT normalize intermediate quaternion products here.
        val out = multiplyQuaternionWxyzRaw(multiplyQuaternionWxyzRaw(q, qv), qConj)
        return doubleArrayOf(out[1], out[2], out[3])
    }

    private fun multiplyQuaternionWxyz(a: DoubleArray, b: DoubleArray): DoubleArray {
        return normalizeQuaternionWxyz(multiplyQuaternionWxyzRaw(a, b))
    }

    private fun multiplyQuaternionWxyzRaw(a: DoubleArray, b: DoubleArray): DoubleArray {
        val aw = a[0]
        val ax = a[1]
        val ay = a[2]
        val az = a[3]
        val bw = b[0]
        val bx = b[1]
        val by = b[2]
        val bz = b[3]
        return doubleArrayOf(
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        )
    }

    private fun conjugateQuaternionWxyz(q: DoubleArray): DoubleArray {
        return doubleArrayOf(q[0], -q[1], -q[2], -q[3])
    }

    private fun normalizeQuaternionWxyz(q: DoubleArray): DoubleArray {
        val n = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
        if (n <= 1e-12) return doubleArrayOf(1.0, 0.0, 0.0, 0.0)
        return doubleArrayOf(q[0] / n, q[1] / n, q[2] / n, q[3] / n)
    }

    private fun initOffscreenGlContext() {
        val display = EGL14.eglGetDisplay(EGL14.EGL_DEFAULT_DISPLAY)
        require(display != EGL14.EGL_NO_DISPLAY) { "Unable to get EGL display" }

        val version = IntArray(2)
        require(EGL14.eglInitialize(display, version, 0, version, 1)) { "Unable to initialize EGL" }

        val attribs = intArrayOf(
            EGL14.EGL_RED_SIZE, 8,
            EGL14.EGL_GREEN_SIZE, 8,
            EGL14.EGL_BLUE_SIZE, 8,
            EGL14.EGL_RENDERABLE_TYPE, EGL14.EGL_OPENGL_ES2_BIT,
            EGL14.EGL_NONE,
        )

        val configs = arrayOfNulls<EGLConfig>(1)
        val numConfigs = IntArray(1)
        require(EGL14.eglChooseConfig(display, attribs, 0, configs, 0, 1, numConfigs, 0) && numConfigs[0] > 0) {
            "Unable to choose EGL config"
        }

        val contextAttribs = intArrayOf(
            EGL14.EGL_CONTEXT_CLIENT_VERSION, 2,
            EGL14.EGL_NONE,
        )

        val context = EGL14.eglCreateContext(display, configs[0], EGL14.EGL_NO_CONTEXT, contextAttribs, 0)
        require(context != null && context != EGL14.EGL_NO_CONTEXT) { "Unable to create EGL context" }

        val pbufferAttribs = intArrayOf(
            EGL14.EGL_WIDTH, 1,
            EGL14.EGL_HEIGHT, 1,
            EGL14.EGL_NONE,
        )
        val surface = EGL14.eglCreatePbufferSurface(display, configs[0], pbufferAttribs, 0)
        require(surface != null && surface != EGL14.EGL_NO_SURFACE) { "Unable to create EGL surface" }

        eglDisplay = display
        eglContext = context
        eglSurface = surface

        makeGlCurrent()
    }

    private fun bindArCoreTexture() {
        val tex = IntArray(1)
        GLES20.glGenTextures(1, tex, 0)
        require(tex[0] != 0) { "Unable to create GL texture for ARCore" }
        cameraTextureId = tex[0]
        arSession?.setCameraTextureName(cameraTextureId)
        Log.i(tag, "bindArCoreTexture(): textureId=$cameraTextureId")
    }

    private fun makeGlCurrent() {
        val display = eglDisplay ?: return
        val surface = eglSurface ?: return
        val context = eglContext ?: return
        val ok = EGL14.eglMakeCurrent(display, surface, surface, context)
        if (!ok) {
            val err = EGL14.eglGetError()
            throw IllegalStateException("eglMakeCurrent failed: 0x${Integer.toHexString(err)}")
        }
    }

    private fun releaseOffscreenGlContext() {
        try {
            if (cameraTextureId > 0) {
                GLES20.glDeleteTextures(1, intArrayOf(cameraTextureId), 0)
            }
            cameraTextureId = -1
        } catch (_: Throwable) {
            // ignore
        }

        val display = eglDisplay
        val surface = eglSurface
        val context = eglContext

        if (display != null && display != EGL14.EGL_NO_DISPLAY) {
            try {
                EGL14.eglMakeCurrent(display, EGL14.EGL_NO_SURFACE, EGL14.EGL_NO_SURFACE, EGL14.EGL_NO_CONTEXT)
            } catch (_: Throwable) {
                // ignore
            }
            if (surface != null && surface != EGL14.EGL_NO_SURFACE) {
                EGL14.eglDestroySurface(display, surface)
            }
            if (context != null && context != EGL14.EGL_NO_CONTEXT) {
                EGL14.eglDestroyContext(display, context)
            }
            EGL14.eglTerminate(display)
        }

        eglDisplay = null
        eglSurface = null
        eglContext = null
    }

    private fun runOnCameraThreadBlocking(action: () -> Unit) {
        val handler = cameraHandler ?: throw IllegalStateException("Camera handler not initialized")
        val latch = CountDownLatch(1)
        var error: Throwable? = null

        handler.post {
            try {
                action()
            } catch (t: Throwable) {
                error = t
            } finally {
                latch.countDown()
            }
        }

        val completed = latch.await(5, TimeUnit.SECONDS)
        if (!completed) {
            throw IllegalStateException("Timed out waiting for camera thread action")
        }
        error?.let { throw it }
    }
}
