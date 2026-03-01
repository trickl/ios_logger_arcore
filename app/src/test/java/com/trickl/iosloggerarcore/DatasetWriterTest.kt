package com.trickl.iosloggerarcore

import org.junit.Assert.assertTrue
import org.junit.Test
import java.io.File

class DatasetWriterTest {
    @Test
    fun `writer emits frames and poses files`() {
        val tempRoot = createTempDir(prefix = "ios_logger_arcore_test_")
        try {
            val writer = DatasetWriter(tempRoot)
            writer.open()
            writer.writeFrame(1.0, 0, CameraIntrinsics(100.0, 100.0, 50.0, 50.0))
            writer.writePose(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
            writer.close()

            val frames = File(writer.datasetDir, "Frames.txt")
            val poses = File(writer.datasetDir, "ARposes.txt")
            assertTrue(frames.exists())
            assertTrue(poses.exists())
            assertTrue(frames.readText().contains("1.000000,0,100.000000,100.000000,50.000000,50.000000"))
            assertTrue(poses.readText().contains("1.000000,0.000000,0.000000,0.000000,1.000000,0.000000,0.000000,0.000000"))
        } finally {
            tempRoot.deleteRecursively()
        }
    }
}
