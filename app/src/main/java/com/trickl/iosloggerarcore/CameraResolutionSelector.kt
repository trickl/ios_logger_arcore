package com.trickl.iosloggerarcore

import android.content.Context
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraManager
import android.media.MediaRecorder
import android.util.Size

object CameraResolutionSelector {
    fun selectFullPipelineResolution(context: Context): Size {
        return try {
            val cameraManager = context.getSystemService(Context.CAMERA_SERVICE) as CameraManager
            val cameraId = chooseBackCameraId(cameraManager) ?: return Size(1920, 1080)
            val chars = cameraManager.getCameraCharacteristics(cameraId)
            val map = chars.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)
                ?: return Size(1920, 1080)

            val mediaSizes = map.getOutputSizes(MediaRecorder::class.java)
            if (mediaSizes.isNullOrEmpty()) {
                Size(1920, 1080)
            } else {
                mediaSizes.maxByOrNull { it.width.toLong() * it.height.toLong() } ?: Size(1920, 1080)
            }
        } catch (_: Throwable) {
            Size(1920, 1080)
        }
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
