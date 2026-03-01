package com.trickl.iosloggerarcore

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import java.util.concurrent.atomic.AtomicReference

class PoseSampler(context: Context) : SensorEventListener {
    private val sensorManager = context.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    private val rotationSensor: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)

    private val latestQuat = AtomicReference(floatArrayOf(1f, 0f, 0f, 0f))

    fun start() {
        rotationSensor?.let {
            sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME)
        }
    }

    fun stop() {
        sensorManager.unregisterListener(this)
    }

    fun currentQuaternionWxyz(): FloatArray = latestQuat.get().copyOf()

    fun currentPoseSample(timestampSeconds: Double): PoseSample {
        val q = currentQuaternionWxyz()
        return PoseSample(
            timestampSeconds = timestampSeconds,
            tx = 0.0,
            ty = 0.0,
            tz = 0.0,
            qw = q[0].toDouble(),
            qx = q[1].toDouble(),
            qy = q[2].toDouble(),
            qz = q[3].toDouble(),
        )
    }

    override fun onSensorChanged(event: SensorEvent) {
        if (event.sensor.type != Sensor.TYPE_ROTATION_VECTOR) return
        val q = FloatArray(4)
        SensorManager.getQuaternionFromVector(q, event.values)
        // Android order from getQuaternionFromVector: [w, x, y, z]
        latestQuat.set(floatArrayOf(q[0], q[1], q[2], q[3]))
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit
}
