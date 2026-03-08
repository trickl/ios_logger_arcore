package com.trickl.iosloggerarcore

import android.app.Activity
import android.content.Context
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
        STABLE_TRACKING,
        RECOVERING,
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
    private var previousExportedPose: PoseSample? = null
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

    private companion object {
        private const val FIRST_PRINCIPLES_POSE_VALIDATION_MODE = false
        private const val ENABLE_CONTINUITY_REANCHOR = !FIRST_PRINCIPLES_POSE_VALIDATION_MODE
        private const val ENABLE_ANCHOR_COMPARISON = !FIRST_PRINCIPLES_POSE_VALIDATION_MODE
        private const val ALLOW_JUMP_ONLY_REANCHOR = false

        private const val HARD_TRANSLATION_RESET_M = 0.35
        private const val HARD_ROTATION_RESET_DEG = 20.0
        private const val COMBINED_TRANSLATION_RESET_M = 0.15
        private const val COMBINED_ROTATION_RESET_DEG = 8.0
        private const val POSE_INSTRUMENTATION_LOG_EVERY_FRAME = true
        private const val STABLE_TRACKING_MIN_FRAMES = 12
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
            if (poseMode == PoseMode.LEGACY_STITCH) {
                val frame = frameCounter.getAndIncrement()
                datasetWriter.writeFrame(ts, frame, null)
                if (!firstFrameEventWritten) {
                    datasetWriter.writeEvent(
                        tsUnixSeconds = ts,
                        event = "first_frame_seen",
                        details = "frameIndex=$frame,sourceFrameIndex=$sourceFrame"
                    )
                    firstFrameEventWritten = true
                }
            }
            onTrackingState("PAUSED")
            if (errors % 10 == 0) {
                Log.w(tag, "processFrame(): session.update failed $errors times (${e::class.java.simpleName}: ${e.message})")
            }
            if (errors > 60) {
                onStatus("Stopped: ARCore update repeatedly failed (${e::class.java.simpleName})")
                Log.e(tag, "processFrame(): stopping after repeated update failures", e)
                stopInternal("arcore update failure")
            }
            return
        }

        updateErrorCounter.set(0)

        val camera = arFrame.camera
        onTrackingState(camera.trackingState.name)

        val intrinsics = try {
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

            mapIntrinsicsToOutputResolution(
                intrinsics = rawIntrinsics,
                srcWidth = srcDims[0],
                srcHeight = srcDims[1],
                dstWidth = resolution.width,
                dstHeight = resolution.height,
            )
        } catch (_: Throwable) {
            null
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
        if (camera.trackingState != TrackingState.TRACKING) {
            val elapsedMs = if (recordingStartElapsedNanos == 0L) 0L
            else (SystemClock.elapsedRealtimeNanos() - recordingStartElapsedNanos) / 1_000_000L

            if (hadTracking) {
                onStatus("Stopped: ARCore tracking lost (${camera.trackingState})")
                Log.w(tag, "processFrame(): tracking lost after capture start (${camera.trackingState})")
                stopInternal("tracking lost")
                return
            }

            if (!hadTracking && elapsedMs < 10_000L) {
                onStatus("Waiting for ARCore tracking... (${camera.trackingState})")
                missingPoseCounter.set(0)
                return
            }

            val misses = missingPoseCounter.incrementAndGet()
            if (misses > 300) {
                onStatus("Stopped: ARCore tracking not active (${camera.trackingState})")
                Log.w(
                    tag,
                    "processFrame(): tracking not active for too long (${camera.trackingState}), misses=$misses, elapsedMs=$elapsedMs"
                )
                stopInternal("tracking lost")
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
            if (hadTracking) {
                onStatus("Stopped: capture anchor tracking lost")
                Log.w(tag, "processFrame(): capture anchor tracking lost after capture start")
                stopInternal("anchor tracking lost")
                return
            }

            val misses = missingPoseCounter.incrementAndGet()
            onStatus("Waiting for capture anchor tracking...")
            if (misses > 300) {
                onStatus("Stopped: capture anchor tracking not active")
                Log.w(tag, "processFrame(): anchor tracking not active for too long, misses=$misses")
                stopInternal("anchor tracking lost")
            }
            return
        }

        hadTracking = true
        missingPoseCounter.set(0)

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

        datasetWriter.writePose(
            exportedPose.timestampSeconds,
            exportedPose.tx,
            exportedPose.ty,
            exportedPose.tz,
            exportedPose.qw,
            exportedPose.qx,
            exportedPose.qy,
            exportedPose.qz,
        )

        if (!firstPoseEventWritten) {
            datasetWriter.writeEvent(
                tsUnixSeconds = exportedPose.timestampSeconds,
                event = "first_pose_written",
                details = String.format(
                    Locale.US,
                    "frameIndex=%d,tx=%.6f,ty=%.6f,tz=%.6f,qw=%.6f,qx=%.6f,qy=%.6f,qz=%.6f",
                    frame,
                    exportedPose.tx,
                    exportedPose.ty,
                    exportedPose.tz,
                    exportedPose.qw,
                    exportedPose.qx,
                    exportedPose.qy,
                    exportedPose.qz,
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

        if (anchorCreateCooldownFrames > 0) {
            anchorCreateCooldownFrames -= 1
        }

        if (camera.trackingState != TrackingState.TRACKING) {
            if (previousTrackingState == TrackingState.TRACKING) {
                pendingRelocalization = true
                Log.w(tag, "Tracking transitioned TRACKING -> ${camera.trackingState}; will re-anchor on recovery")
            }
            transitionTrackingPipelineState(TrackingPipelineState.RECOVERING, ts)
            trackingConsecutiveFrames = 0
            recoveringConsecutiveFrames += 1
            previousTrackingState = camera.trackingState

            val elapsedMs = if (recordingStartElapsedNanos == 0L) 0L
            else (SystemClock.elapsedRealtimeNanos() - recordingStartElapsedNanos) / 1_000_000L

            if (!hadTracking && elapsedMs < 10_000L) {
                onStatus("Waiting for ARCore tracking... (${camera.trackingState})")
                missingPoseCounter.set(0)
                return
            }

            val misses = missingPoseCounter.incrementAndGet()
            val threshold = if (hadTracking) RECOVERING_STOP_FRAMES else 300
            if (misses > threshold) {
                onStatus("Stopped: ARCore tracking not active (${camera.trackingState})")
                Log.w(
                    tag,
                    "processFrame(): tracking not active for too long (${camera.trackingState}), misses=$misses, elapsedMs=$elapsedMs"
                )
                stopInternal("tracking lost")
            }
            return
        }

        val recoveredFromNonTracking = previousTrackingState != TrackingState.TRACKING
        previousTrackingState = TrackingState.TRACKING

        trackingConsecutiveFrames += 1
        recoveringConsecutiveFrames = 0

        if (!hadTracking && trackingConsecutiveFrames >= STABLE_TRACKING_MIN_FRAMES) {
            transitionTrackingPipelineState(TrackingPipelineState.STABLE_TRACKING, ts)
        } else if (hadTracking && recoveredFromNonTracking) {
            transitionTrackingPipelineState(TrackingPipelineState.RECOVERING, ts)
        }

        hadTracking = true
        missingPoseCounter.set(0)

        val pose = camera.pose
        val q = pose.rotationQuaternion // x, y, z, w
        val poseTsRaw = ts
        val poseTs = makePoseTimestampStrictlyIncreasing(poseTsRaw)

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
        if (ENABLE_ANCHOR_COMPARISON && comparisonAnchor != null && comparisonAnchor.trackingState == TrackingState.TRACKING) {
            val poseRelativeToAnchor = comparisonAnchor.pose.inverse().compose(pose)
            val aq = poseRelativeToAnchor.rotationQuaternion // x, y, z, w
            val anchorExportedPose = ArKitConventionMapper.mapPose(
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

        val rawDelta = computePoseDelta(previousRawArKitPose, arKitPose)
        val jumpSuspected = isLikelyWorldReset(rawDelta)
        val recoverySettled = trackingConsecutiveFrames >= STABLE_TRACKING_MIN_FRAMES
        val recoveryResetCandidate = (pendingRelocalization && recoverySettled) ||
            (recoveredFromNonTracking && recoverySettled)
        val jumpOnlyResetCandidate = jumpSuspected && !pendingRelocalization && !recoveredFromNonTracking
        val resetDetected = ENABLE_CONTINUITY_REANCHOR && (
            recoveryResetCandidate ||
                (ALLOW_JUMP_ONLY_REANCHOR && jumpOnlyResetCandidate)
            )

        if (resetDetected && previousExportedPose != null) {
            worldReanchorTransform = computeAlignmentTransform(
                fromCurrentRawPose = arKitPose,
                toTargetExportPose = previousExportedPose!!,
            )
            val resets = relocalizationCounter.incrementAndGet()
            Log.w(
                tag,
                "Pose continuity reset detected (#$resets): sourceFrame=${arFrameTimestampNanos}, " +
                    "trackingRecovered=$recoveredFromNonTracking, pendingRelocalization=$pendingRelocalization, " +
                    "jumpSuspected=$jumpSuspected, rawDelta_m=${rawDelta?.translationMeters}, " +
                    "rawDelta_deg=${rawDelta?.rotationDegrees}"
            )
            transitionTrackingPipelineState(TrackingPipelineState.STABLE_TRACKING, exportedPoseTsForTransition(poseTs))
            datasetWriter.writeEvent(
                tsUnixSeconds = poseTs,
                event = "tracking_relocalized",
                details = String.format(
                    Locale.US,
                    "relocalization_count=%d,recoveredFromNonTracking=%s,pendingRelocalization=%s,jumpSuspected=%s,jumpOnlyResetCandidate=%s,allowJumpOnly=%s,pose_mode=%s",
                    resets,
                    recoveredFromNonTracking,
                    pendingRelocalization,
                    jumpSuspected,
                    jumpOnlyResetCandidate,
                    ALLOW_JUMP_ONLY_REANCHOR,
                    poseMode.name,
                )
            )
        }
        if (recoverySettled) {
            pendingRelocalization = false
            transitionTrackingPipelineState(TrackingPipelineState.STABLE_TRACKING, poseTs)
        }

        val exportedPose = if (ENABLE_CONTINUITY_REANCHOR) {
            applyTransform(worldReanchorTransform, arKitPose)
        } else {
            arKitPose
        }
        val exportedDelta = computePoseDelta(previousExportedPose, exportedPose)
        val exportedTranslationBreakdown = computeTranslationDeltaBreakdown(previousExportedPose, exportedPose)

        logPoseInstrumentation(
            sourceFrameIndex = sourceFrame,
            sourceFrameTimestampNanos = arFrameTimestampNanos,
            trackingState = camera.trackingState,
            relocalizationOrReset = resetDetected,
            rawDelta = rawDelta,
            exportedDelta = exportedDelta,
            exportedTranslationBreakdown = exportedTranslationBreakdown,
        )

        datasetWriter.writePose(
            exportedPose.timestampSeconds,
            exportedPose.tx,
            exportedPose.ty,
            exportedPose.tz,
            exportedPose.qw,
            exportedPose.qx,
            exportedPose.qy,
            exportedPose.qz,
        )

        if (!firstPoseEventWritten) {
            datasetWriter.writeEvent(
                tsUnixSeconds = exportedPose.timestampSeconds,
                event = "first_pose_written",
                details = String.format(
                    Locale.US,
                    "frameIndex=%d,tx=%.6f,ty=%.6f,tz=%.6f,qw=%.6f,qx=%.6f,qy=%.6f,qz=%.6f",
                    frame,
                    exportedPose.tx,
                    exportedPose.ty,
                    exportedPose.tz,
                    exportedPose.qw,
                    exportedPose.qx,
                    exportedPose.qy,
                    exportedPose.qz,
                )
            )
            firstPoseEventWritten = true
        }

        previousRawArKitPose = arKitPose
        previousExportedPose = exportedPose
    }

    private fun exportedPoseTsForTransition(poseTs: Double): Double = poseTs

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

        cameraManager.openCamera(cameraId, wrappedDeviceCallback, cameraHandler)
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
            previousExportedPose = null
            worldReanchorTransform = null
            captureAnchor = null
            anchorComparisonDisabled = false
            if (!ENABLE_ANCHOR_COMPARISON) {
                anchorComparisonDisabled = true
            }
            trackingPipelineState = TrackingPipelineState.WARMUP
            trackingConsecutiveFrames = 0
            recoveringConsecutiveFrames = 0
            previousAnchorComparisonPose = null
            anchorHealthyStreak = 0
            anchorUnhealthyStreak = 0
            anchorCreateCooldownFrames = 0
            firstAnchorEventWritten = false
            firstFrameEventWritten = false
            firstPoseEventWritten = false
            datasetWriter.writeEvent(
                tsUnixSeconds = System.currentTimeMillis() / 1000.0,
                event = "engine_started",
                details = "shared_camera=true,pose_mode=${poseMode.name}"
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
