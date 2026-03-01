package com.trickl.iosloggerarcore

import android.app.Activity
import android.content.Context
import com.google.ar.core.ArCoreApk
import com.google.ar.core.Camera
import com.google.ar.core.Config
import com.google.ar.core.Session
import com.google.ar.core.TrackingState
import com.google.ar.core.exceptions.CameraNotAvailableException
import com.google.ar.core.exceptions.UnavailableException

class ArCorePoseSampler(
    private val context: Context,
    private val bootToUnixOffsetSeconds: Double,
) {
    private var session: Session? = null
    private var lastFailureReason: String = "ARCore not started"

    fun getLastFailureReason(): String = lastFailureReason

    fun start(): Boolean {
        val activity = context as? Activity ?: return false

        return try {
            val availability = ArCoreApk.getInstance().checkAvailability(activity)
            if (!availability.isSupported) {
                lastFailureReason = "Device does not support ARCore"
                return false
            }

            val install = ArCoreApk.getInstance().requestInstall(activity, true)
            if (install == ArCoreApk.InstallStatus.INSTALL_REQUESTED) {
                lastFailureReason = "ARCore install/update requested; reopen capture after installation completes"
                return false
            }

            val created = Session(activity)
            val config = Config(created).apply {
                updateMode = Config.UpdateMode.LATEST_CAMERA_IMAGE
            }
            created.configure(config)
            created.resume()
            session = created
            lastFailureReason = ""
            true
        } catch (e: UnavailableException) {
            lastFailureReason = "ARCore unavailable: ${e.message ?: e::class.java.simpleName}"
            false
        } catch (e: CameraNotAvailableException) {
            lastFailureReason = "ARCore camera unavailable: ${e.message ?: e::class.java.simpleName}"
            false
        } catch (e: SecurityException) {
            lastFailureReason = "Camera permission missing or denied"
            false
        } catch (e: IllegalStateException) {
            lastFailureReason = "ARCore illegal state: ${e.message ?: e::class.java.simpleName}"
            false
        }
    }

    fun stop() {
        session?.pause()
        session?.close()
        session = null
    }

    fun pollPose(): PoseSample? {
        val s = session ?: return null
        return try {
            val frame = s.update()
            val camera = frame.camera
            if (camera.trackingState != TrackingState.TRACKING) {
                lastFailureReason = "ARCore tracking not active (${camera.trackingState})"
                return null
            }

            val p = camera.pose
            val q = p.rotationQuaternion // x, y, z, w
            val ts = TimeUtils.toUnixSeconds(frame.timestamp, bootToUnixOffsetSeconds)

            ArKitConventionMapper.mapPose(
                timestampSeconds = ts,
                tx = p.tx().toDouble(),
                ty = p.ty().toDouble(),
                tz = p.tz().toDouble(),
                qx = q[0].toDouble(),
                qy = q[1].toDouble(),
                qz = q[2].toDouble(),
                qw = q[3].toDouble(),
            )
        } catch (e: Exception) {
            lastFailureReason = "ARCore frame update failed: ${e.message ?: e::class.java.simpleName}"
            null
        }
    }
}
