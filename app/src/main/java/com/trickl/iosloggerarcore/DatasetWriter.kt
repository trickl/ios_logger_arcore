package com.trickl.iosloggerarcore

import java.io.BufferedWriter
import java.io.File
import java.io.FileWriter
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

class DatasetWriter(baseDir: File) {
    private val folderName = SimpleDateFormat("yyyy-MM-dd'T'HH-mm-ss", Locale.US).format(Date())
    val datasetDir: File = File(baseDir, folderName)

    val videoFile: File = File(datasetDir, "Frames.m4v")

    private lateinit var framesWriter: BufferedWriter
    private lateinit var posesWriter: BufferedWriter
    private lateinit var accelWriter: BufferedWriter
    private lateinit var gyroWriter: BufferedWriter
    private lateinit var gpsWriter: BufferedWriter
    private lateinit var headWriter: BufferedWriter
    private lateinit var motionWriter: BufferedWriter
    private lateinit var motArhWriter: BufferedWriter
    private lateinit var motMagnFullWriter: BufferedWriter
    private lateinit var magnetWriter: BufferedWriter
    private lateinit var eventsWriter: BufferedWriter

    fun open() {
        if (!datasetDir.exists()) {
            datasetDir.mkdirs()
        }
        framesWriter = BufferedWriter(FileWriter(File(datasetDir, "Frames.txt"), false))
        posesWriter = BufferedWriter(FileWriter(File(datasetDir, "ARposes.txt"), false))
        accelWriter = BufferedWriter(FileWriter(File(datasetDir, "Accel.txt"), false))
        gyroWriter = BufferedWriter(FileWriter(File(datasetDir, "Gyro.txt"), false))
        gpsWriter = BufferedWriter(FileWriter(File(datasetDir, "GPS.txt"), false))
        headWriter = BufferedWriter(FileWriter(File(datasetDir, "Head.txt"), false))
        motionWriter = BufferedWriter(FileWriter(File(datasetDir, "Motion.txt"), false))
        motArhWriter = BufferedWriter(FileWriter(File(datasetDir, "MotARH.txt"), false))
        motMagnFullWriter = BufferedWriter(FileWriter(File(datasetDir, "MotMagnFull.txt"), false))
        magnetWriter = BufferedWriter(FileWriter(File(datasetDir, "Magnet.txt"), false))
        eventsWriter = BufferedWriter(FileWriter(File(datasetDir, "CaptureEvents.txt"), false))
    }

    @Synchronized
    fun writeFrame(ts: Double, frame: Long, intrinsics: CameraIntrinsics?) {
        if (intrinsics == null) {
            framesWriter.write("%.6f,%d\n".format(Locale.US, ts, frame))
        } else {
            framesWriter.write(
                "%.6f,%d,%.6f,%.6f,%.6f,%.6f\n".format(
                    Locale.US,
                    ts,
                    frame,
                    intrinsics.fx,
                    intrinsics.fy,
                    intrinsics.cx,
                    intrinsics.cy
                )
            )
        }
    }

    @Synchronized
    fun writePose(ts: Double, tx: Double, ty: Double, tz: Double, qw: Double, qx: Double, qy: Double, qz: Double) {
        posesWriter.write(
            "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n".format(
                Locale.US,
                ts,
                tx,
                ty,
                tz,
                qw,
                qx,
                qy,
                qz
            )
        )
    }

    @Synchronized
    fun writeAccel(ts: Double, ax: Double, ay: Double, az: Double) {
        accelWriter.write("%.6f,%.6f,%.6f,%.6f\n".format(Locale.US, ts, ax, ay, az))
    }

    @Synchronized
    fun writeGyro(ts: Double, gx: Double, gy: Double, gz: Double) {
        gyroWriter.write("%.6f,%.6f,%.6f,%.6f\n".format(Locale.US, ts, gx, gy, gz))
    }

    @Synchronized
    fun writeGps(
        ts: Double,
        latitude: Double,
        longitude: Double,
        horizontalAccuracy: Double,
        altitude: Double,
        verticalAccuracy: Double,
        floorLevel: Int,
        course: Double,
        speed: Double,
    ) {
        gpsWriter.write(
            "%.6f,%.8f,%.8f,%.6f,%.6f,%.6f,%d,%.6f,%.6f\n".format(
                Locale.US,
                ts,
                latitude,
                longitude,
                horizontalAccuracy,
                altitude,
                verticalAccuracy,
                floorLevel,
                course,
                speed,
            )
        )
    }

    @Synchronized
    fun writeHead(ts: Double, trueHeading: Double, magneticHeading: Double, headingAccuracy: Double) {
        headWriter.write(
            "%.6f,%.6f,%.6f,%.6f\n".format(
                Locale.US,
                ts,
                trueHeading,
                magneticHeading,
                headingAccuracy,
            )
        )
    }

    @Synchronized
    fun writeMotion(ts: Double, qw: Double, qx: Double, qy: Double, qz: Double) {
        motionWriter.write(
            "%.6f,%.6f,%.6f,%.6f,%.6f\n".format(
                Locale.US,
                ts,
                qw,
                qx,
                qy,
                qz,
            )
        )
    }

    @Synchronized
    fun writeMotArh(
        ts: Double,
        rotX: Double,
        rotY: Double,
        rotZ: Double,
        gravX: Double,
        gravY: Double,
        gravZ: Double,
        userAccX: Double,
        userAccY: Double,
        userAccZ: Double,
        motionHeading: Double,
    ) {
        motArhWriter.write(
            "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n".format(
                Locale.US,
                ts,
                rotX,
                rotY,
                rotZ,
                gravX,
                gravY,
                gravZ,
                userAccX,
                userAccY,
                userAccZ,
                motionHeading,
            )
        )
    }

    @Synchronized
    fun writeMotMagnFull(ts: Double, magnX: Double, magnY: Double, magnZ: Double, magnAccuracy: Int) {
        motMagnFullWriter.write(
            "%.6f,%.6f,%.6f,%.6f,%d\n".format(
                Locale.US,
                ts,
                magnX,
                magnY,
                magnZ,
                magnAccuracy,
            )
        )
    }

    @Synchronized
    fun writeMagnet(ts: Double, magnX: Double, magnY: Double, magnZ: Double) {
        magnetWriter.write("%.6f,%.6f,%.6f,%.6f\n".format(Locale.US, ts, magnX, magnY, magnZ))
    }

    @Synchronized
    fun writeEvent(tsUnixSeconds: Double, event: String, details: String = "") {
        eventsWriter.write("%.6f,%s,%s\n".format(Locale.US, tsUnixSeconds, event, details))
    }

    @Synchronized
    fun close() {
        if (::framesWriter.isInitialized) {
            framesWriter.flush()
            framesWriter.close()
        }
        if (::posesWriter.isInitialized) {
            posesWriter.flush()
            posesWriter.close()
        }
        if (::accelWriter.isInitialized) {
            accelWriter.flush()
            accelWriter.close()
        }
        if (::gyroWriter.isInitialized) {
            gyroWriter.flush()
            gyroWriter.close()
        }
        if (::gpsWriter.isInitialized) {
            gpsWriter.flush()
            gpsWriter.close()
        }
        if (::headWriter.isInitialized) {
            headWriter.flush()
            headWriter.close()
        }
        if (::motionWriter.isInitialized) {
            motionWriter.flush()
            motionWriter.close()
        }
        if (::motArhWriter.isInitialized) {
            motArhWriter.flush()
            motArhWriter.close()
        }
        if (::motMagnFullWriter.isInitialized) {
            motMagnFullWriter.flush()
            motMagnFullWriter.close()
        }
        if (::magnetWriter.isInitialized) {
            magnetWriter.flush()
            magnetWriter.close()
        }
        if (::eventsWriter.isInitialized) {
            eventsWriter.flush()
            eventsWriter.close()
        }
    }
}
