package com.trickl.iosloggerarcore

import android.content.Context
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraManager
import android.media.MediaRecorder
import android.util.Size

object CameraResolutionSelector {
    private val INTRINSICS_AB_TEST_RESOLUTION = Size(1920, 1440)

    fun selectIntrinsicsAbTestResolution(context: Context): Size {
        return selectExactMediaRecorderResolution(
            context = context,
            target = INTRINSICS_AB_TEST_RESOLUTION,
        )
    }

    fun selectFullPipelineResolution(context: Context): Size {
        val cameraManager = context.getSystemService(Context.CAMERA_SERVICE) as CameraManager
        val cameraId = chooseBackCameraId(cameraManager)
            ?: throw IllegalStateException("Failed to detect camera resolution: no camera ID available")

        val chars = cameraManager.getCameraCharacteristics(cameraId)
        val map = chars.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)
            ?: throw IllegalStateException(
                "Failed to detect camera resolution: SCALER_STREAM_CONFIGURATION_MAP unavailable for cameraId=$cameraId"
            )

        val mediaSizes = map.getOutputSizes(MediaRecorder::class.java)
            ?: throw IllegalStateException(
                "Failed to detect camera resolution: MediaRecorder output sizes unavailable for cameraId=$cameraId"
            )

        return mediaSizes.maxByOrNull { it.width.toLong() * it.height.toLong() }
            ?: throw IllegalStateException(
                "Failed to detect camera resolution: empty MediaRecorder output sizes for cameraId=$cameraId"
            )
    }

    fun selectExactMediaRecorderResolution(context: Context, target: Size): Size {
        val cameraManager = context.getSystemService(Context.CAMERA_SERVICE) as CameraManager
        val cameraId = chooseBackCameraId(cameraManager)
            ?: throw IllegalStateException("Failed to detect camera resolution: no camera ID available")

        val chars = cameraManager.getCameraCharacteristics(cameraId)
        val map = chars.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)
            ?: throw IllegalStateException(
                "Failed to detect camera resolution: SCALER_STREAM_CONFIGURATION_MAP unavailable for cameraId=$cameraId"
            )

        val mediaSizes = map.getOutputSizes(MediaRecorder::class.java)
            ?: throw IllegalStateException(
                "Failed to detect camera resolution: MediaRecorder output sizes unavailable for cameraId=$cameraId"
            )

        val exact = mediaSizes.firstOrNull { it.width == target.width && it.height == target.height }
        if (exact != null) return exact

        val supported = mediaSizes
            .sortedWith(compareByDescending<Size> { it.width.toLong() * it.height.toLong() })
            .take(10)
            .joinToString(separator = ",") { "${it.width}x${it.height}" }

        throw IllegalStateException(
            "Requested capture resolution ${target.width}x${target.height} is not supported for MediaRecorder on cameraId=$cameraId. Top supported: $supported"
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
