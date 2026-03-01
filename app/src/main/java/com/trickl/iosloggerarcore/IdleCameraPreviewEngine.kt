package com.trickl.iosloggerarcore

import android.content.Context
import android.hardware.camera2.CameraCaptureSession
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraDevice
import android.hardware.camera2.CameraManager
import android.hardware.camera2.CaptureRequest
import android.os.Handler
import android.os.HandlerThread
import android.util.Log
import android.util.Size
import android.view.Surface

class IdleCameraPreviewEngine(
    private val context: Context,
    private val resolution: Size,
    private val previewSurface: Surface,
    private val onStatus: (String) -> Unit,
) {
    private val tag = "IdleCameraPreview"

    private var cameraThread: HandlerThread? = null
    private var cameraHandler: Handler? = null
    private var cameraDevice: CameraDevice? = null
    private var captureSession: CameraCaptureSession? = null

    fun start(): Boolean {
        return try {
            cameraThread = HandlerThread("idle-camera-preview").also { it.start() }
            cameraHandler = Handler(cameraThread!!.looper)

            val cameraManager = context.getSystemService(Context.CAMERA_SERVICE) as CameraManager
            val cameraId = chooseBackCameraId(cameraManager)
            if (cameraId == null) {
                onStatus("Preview unavailable: no back camera")
                stop()
                return false
            }

            cameraManager.openCamera(
                cameraId,
                object : CameraDevice.StateCallback() {
                    override fun onOpened(device: CameraDevice) {
                        cameraDevice = device
                        try {
                            previewSurface.isValid
                            createSession(device)
                        } catch (t: Throwable) {
                            Log.e(tag, "onOpened/createSession failed", t)
                            onStatus("Preview unavailable: ${t.message ?: t::class.java.simpleName}")
                            stop()
                        }
                    }

                    override fun onDisconnected(device: CameraDevice) {
                        Log.w(tag, "Camera disconnected")
                        stop()
                    }

                    override fun onError(device: CameraDevice, error: Int) {
                        Log.e(tag, "Camera error: $error")
                        onStatus("Preview camera error: $error")
                        stop()
                    }
                },
                cameraHandler
            )

            true
        } catch (t: Throwable) {
            Log.e(tag, "start() failed", t)
            onStatus("Preview start failed: ${t.message ?: t::class.java.simpleName}")
            stop()
            false
        }
    }

    fun stop() {
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
            cameraThread?.quitSafely()
        } catch (_: Throwable) {
            // ignore
        } finally {
            cameraThread = null
            cameraHandler = null
        }
    }

    private fun createSession(device: CameraDevice) {
        device.createCaptureSession(
            listOf(previewSurface),
            object : CameraCaptureSession.StateCallback() {
                override fun onConfigured(session: CameraCaptureSession) {
                    captureSession = session
                    try {
                        val request = device.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW).apply {
                            addTarget(previewSurface)
                        }.build()
                        session.setRepeatingRequest(request, null, cameraHandler)
                        Log.i(tag, "Idle preview started (${resolution.width}x${resolution.height})")
                    } catch (t: Throwable) {
                        Log.e(tag, "Failed to start repeating preview", t)
                        onStatus("Preview start failed: ${t.message ?: t::class.java.simpleName}")
                        stop()
                    }
                }

                override fun onConfigureFailed(session: CameraCaptureSession) {
                    Log.e(tag, "Capture session configure failed")
                    onStatus("Preview configure failed")
                    stop()
                }
            },
            cameraHandler
        )
    }

    private fun chooseBackCameraId(cameraManager: CameraManager): String? {
        cameraManager.cameraIdList.forEach { id ->
            val chars = cameraManager.getCameraCharacteristics(id)
            if (chars.get(CameraCharacteristics.LENS_FACING) == CameraCharacteristics.LENS_FACING_BACK) {
                return id
            }
        }
        return cameraManager.cameraIdList.firstOrNull()
    }
}
