package com.trickl.iosloggerarcore

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.os.Build
import android.os.Looper
import androidx.core.content.ContextCompat
import kotlin.math.PI

class MotionLocationSampler(
    private val context: Context,
    private val bootToUnixOffsetSeconds: Double,
    private val writerProvider: () -> DatasetWriter?,
) : SensorEventListener, LocationListener {

    private val sensorManager = context.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    private val locationManager = context.getSystemService(Context.LOCATION_SERVICE) as LocationManager

    private val accelSensor: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
    private val gyroSensor: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
    private val magnetSensor: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)
    private val rotationVectorSensor: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)
    private val gravitySensor: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY)
    private val linearAccelSensor: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION)

    private val accel = FloatArray(3)
    private val magnet = FloatArray(3)
    private val gravity = FloatArray(3)
    private val userAccel = FloatArray(3)
    private val gyro = FloatArray(3)

    private var hasAccel = false
    private var hasMagnet = false

    private var latestLocation: Location? = null
    private var latestDeclinationDeg: Float = 0f
    private var latestHeadingAccuracyDeg: Double = -1.0

    fun start() {
        accelSensor?.let { sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME) }
        gyroSensor?.let { sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME) }
        magnetSensor?.let { sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME) }
        rotationVectorSensor?.let { sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME) }
        gravitySensor?.let { sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME) }
        linearAccelSensor?.let { sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME) }

        val hasFine = ContextCompat.checkSelfPermission(context, Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED
        val hasCoarse = ContextCompat.checkSelfPermission(context, Manifest.permission.ACCESS_COARSE_LOCATION) == PackageManager.PERMISSION_GRANTED
        if (!hasFine && !hasCoarse) return

        try {
            if (locationManager.isProviderEnabled(LocationManager.GPS_PROVIDER)) {
                locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 0L, 0f, this, Looper.getMainLooper())
            }
            if (locationManager.isProviderEnabled(LocationManager.NETWORK_PROVIDER)) {
                locationManager.requestLocationUpdates(LocationManager.NETWORK_PROVIDER, 0L, 0f, this, Looper.getMainLooper())
            }
        } catch (_: SecurityException) {
            // Permission race, ignore and continue sensor-only logging.
        }
    }

    fun stop() {
        sensorManager.unregisterListener(this)
        locationManager.removeUpdates(this)
    }

    override fun onSensorChanged(event: SensorEvent) {
        val writer = writerProvider() ?: return
        val ts = TimeUtils.toUnixSeconds(event.timestamp, bootToUnixOffsetSeconds)

        when (event.sensor.type) {
            Sensor.TYPE_ACCELEROMETER -> {
                accel[0] = event.values[0]
                accel[1] = event.values[1]
                accel[2] = event.values[2]
                hasAccel = true

                writer.writeAccel(
                    ts,
                    accel[0].toDouble() / SensorManager.GRAVITY_EARTH,
                    accel[1].toDouble() / SensorManager.GRAVITY_EARTH,
                    accel[2].toDouble() / SensorManager.GRAVITY_EARTH,
                )
            }

            Sensor.TYPE_GYROSCOPE -> {
                gyro[0] = event.values[0]
                gyro[1] = event.values[1]
                gyro[2] = event.values[2]
                writer.writeGyro(ts, gyro[0].toDouble(), gyro[1].toDouble(), gyro[2].toDouble())
                writeMotArh(writer, ts)
            }

            Sensor.TYPE_MAGNETIC_FIELD -> {
                magnet[0] = event.values[0]
                magnet[1] = event.values[1]
                magnet[2] = event.values[2]
                hasMagnet = true

                latestHeadingAccuracyDeg = when (event.accuracy) {
                    SensorManager.SENSOR_STATUS_ACCURACY_HIGH -> 15.0
                    SensorManager.SENSOR_STATUS_ACCURACY_MEDIUM -> 35.0
                    SensorManager.SENSOR_STATUS_ACCURACY_LOW -> 60.0
                    else -> -1.0
                }

                writer.writeMagnet(ts, magnet[0].toDouble(), magnet[1].toDouble(), magnet[2].toDouble())
                writer.writeMotMagnFull(ts, magnet[0].toDouble(), magnet[1].toDouble(), magnet[2].toDouble(), event.accuracy)
                writeHeading(writer, ts)
            }

            Sensor.TYPE_ROTATION_VECTOR -> {
                val q = FloatArray(4)
                SensorManager.getQuaternionFromVector(q, event.values)
                writer.writeMotion(ts, q[0].toDouble(), q[1].toDouble(), q[2].toDouble(), q[3].toDouble())
                writeMotArh(writer, ts)
                writeHeading(writer, ts)
            }

            Sensor.TYPE_GRAVITY -> {
                gravity[0] = event.values[0]
                gravity[1] = event.values[1]
                gravity[2] = event.values[2]
                writeMotArh(writer, ts)
            }

            Sensor.TYPE_LINEAR_ACCELERATION -> {
                userAccel[0] = event.values[0]
                userAccel[1] = event.values[1]
                userAccel[2] = event.values[2]
                writeMotArh(writer, ts)
            }
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit

    override fun onLocationChanged(location: Location) {
        latestLocation = location

        val geomagnetic = android.hardware.GeomagneticField(
            location.latitude.toFloat(),
            location.longitude.toFloat(),
            location.altitude.toFloat(),
            location.time,
        )
        latestDeclinationDeg = geomagnetic.declination

        val ts = location.time / 1000.0
        val writer = writerProvider() ?: return

        val verticalAccuracy = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O && location.hasVerticalAccuracy()) {
            location.verticalAccuracyMeters.toDouble()
        } else {
            0.0
        }

        val floorLevel = 0 // Android Location has no direct floor level equivalent.

        writer.writeGps(
            ts = ts,
            latitude = location.latitude,
            longitude = location.longitude,
            horizontalAccuracy = if (location.hasAccuracy()) location.accuracy.toDouble() else 0.0,
            altitude = if (location.hasAltitude()) location.altitude else 0.0,
            verticalAccuracy = verticalAccuracy,
            floorLevel = floorLevel,
            course = if (location.hasBearing()) location.bearing.toDouble() else 0.0,
            speed = if (location.hasSpeed()) location.speed.toDouble() else 0.0,
        )

        writeHeading(writer, ts)
    }

    override fun onProviderEnabled(provider: String) = Unit

    override fun onProviderDisabled(provider: String) = Unit

    @Deprecated("Deprecated in Java")
    @Suppress("DEPRECATION")
    override fun onStatusChanged(provider: String?, status: Int, extras: android.os.Bundle?) = Unit

    private fun writeMotArh(writer: DatasetWriter, ts: Double) {
        writer.writeMotArh(
            ts = ts,
            rotX = gyro[0].toDouble(),
            rotY = gyro[1].toDouble(),
            rotZ = gyro[2].toDouble(),
            gravX = gravity[0].toDouble() / SensorManager.GRAVITY_EARTH,
            gravY = gravity[1].toDouble() / SensorManager.GRAVITY_EARTH,
            gravZ = gravity[2].toDouble() / SensorManager.GRAVITY_EARTH,
            userAccX = userAccel[0].toDouble() / SensorManager.GRAVITY_EARTH,
            userAccY = userAccel[1].toDouble() / SensorManager.GRAVITY_EARTH,
            userAccZ = userAccel[2].toDouble() / SensorManager.GRAVITY_EARTH,
            motionHeading = computeTrueHeadingDeg(),
        )
    }

    private fun writeHeading(writer: DatasetWriter, ts: Double) {
        val magneticHeading = computeMagHeadingDeg()
        val trueHeading = normalizeDeg(magneticHeading + latestDeclinationDeg)
        writer.writeHead(ts, trueHeading.toDouble(), magneticHeading.toDouble(), latestHeadingAccuracyDeg)
    }

    private fun computeMagHeadingDeg(): Float {
        if (!hasAccel || !hasMagnet) {
            return latestLocation?.bearing ?: 0f
        }
        val rotationMatrix = FloatArray(9)
        val orientation = FloatArray(3)
        val ok = SensorManager.getRotationMatrix(rotationMatrix, null, accel, magnet)
        if (!ok) return latestLocation?.bearing ?: 0f
        SensorManager.getOrientation(rotationMatrix, orientation)
        val azimuthRad = orientation[0]
        return normalizeDeg((azimuthRad * 180f / PI.toFloat()))
    }

    private fun computeTrueHeadingDeg(): Double {
        val magnetic = computeMagHeadingDeg()
        return normalizeDeg(magnetic + latestDeclinationDeg).toDouble()
    }

    private fun normalizeDeg(v: Float): Float {
        var value = v % 360f
        if (value < 0f) value += 360f
        return value
    }
}
