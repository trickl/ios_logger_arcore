package com.trickl.iosloggerarcore

import android.Manifest
import android.app.Activity
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.animation.animateColorAsState
import androidx.compose.animation.core.FastOutSlowInEasing
import androidx.compose.animation.core.animateDpAsState
import androidx.compose.animation.core.tween
import androidx.compose.foundation.clickable
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.navigationBarsPadding
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.statusBarsPadding
import androidx.compose.foundation.layout.aspectRatio
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Settings
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.core.content.ContextCompat
import com.trickl.iosloggerarcore.ui.theme.IosLoggerArcoreTheme
import kotlin.math.max
import android.view.TextureView

class MainActivity : ComponentActivity() {
    private lateinit var controller: CaptureController

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        controller = CaptureController(this, this)

        setContent {
            IosLoggerArcoreTheme {
                CaptureScreen(controller)
            }
        }
    }

    override fun onDestroy() {
        controller.release()
        super.onDestroy()
    }
}

@Composable
private fun CaptureScreen(controller: CaptureController) {
    val context = LocalContext.current
    val isRecording by controller.isRecording.collectAsState()
    val status by controller.status.collectAsState()
    val trackingState by controller.trackingState.collectAsState()
    val recordedBytes by controller.recordedBytes.collectAsState()
    val datasetPath by controller.datasetPath.collectAsState()

    val fullResolution = remember { CameraResolutionSelector.selectFullPipelineResolution(context) }
    val selected = remember {
        ResolutionOption(
            label = "${fullResolution.width}x${fullResolution.height}",
            size = fullResolution
        )
    }

    val requiredPermissions = remember {
        arrayOf(
            Manifest.permission.CAMERA,
            Manifest.permission.ACCESS_FINE_LOCATION,
            Manifest.permission.ACCESS_COARSE_LOCATION,
        )
    }

    var hasPermission by remember { mutableStateOf(hasRequiredPermissions(context, requiredPermissions)) }
    var pendingStart by remember { mutableStateOf(false) }
    var settingsExpanded by remember { mutableStateOf(false) }

    val previewView = remember { TextureView(context) }

    val permissionLauncher = rememberLauncherForActivityResult(
        contract = ActivityResultContracts.RequestMultiplePermissions()
    ) {
        hasPermission = hasRequiredPermissions(context, requiredPermissions)
        controller.updatePreview(previewView, selected.size, hasPermission)
        if (hasPermission && pendingStart) {
            controller.start(selected.size, previewView)
        }
        pendingStart = false
    }

    LaunchedEffect(selected, hasPermission, isRecording) {
        controller.updatePreview(previewView, selected.size, hasPermission)
    }

    DisposableEffect(Unit) {
        onDispose { if (isRecording) controller.stop() }
    }

    val aspectRatio = selected.size.width.toFloat() / selected.size.height.toFloat()

    Surface(modifier = Modifier.fillMaxSize(), color = Color.Black) {
        Column(
            modifier = Modifier
                .fillMaxSize()
                .navigationBarsPadding()
        ) {
            Box(
                modifier = Modifier
                    .weight(1f)
                    .fillMaxWidth()
                    .background(Color.Black),
                contentAlignment = Alignment.Center
            ) {
                AndroidView(
                    factory = { previewView },
                    modifier = Modifier
                        .fillMaxWidth()
                        .aspectRatio(aspectRatio)
                )

                if (isRecording) {
                    Box(
                        modifier = Modifier
                            .align(Alignment.TopEnd)
                            .statusBarsPadding()
                            .padding(top = 8.dp, end = 16.dp)
                            .size(14.dp)
                            .background(Color.Red, CircleShape)
                    )
                }

                Row(
                    modifier = Modifier
                        .align(Alignment.TopStart)
                        .statusBarsPadding()
                        .padding(horizontal = 16.dp, vertical = 8.dp),
                    verticalAlignment = Alignment.CenterVertically,
                    horizontalArrangement = Arrangement.spacedBy(8.dp)
                ) {
                    Text(
                        text = "EasyCam",
                        color = Color.White,
                        fontWeight = FontWeight.SemiBold,
                        fontSize = 17.sp
                    )

                    Box(
                        modifier = Modifier
                            .size(7.dp)
                            .background(colorForTrackingState(trackingState), CircleShape)
                    )
                    Text(
                        text = trackingState,
                        color = Color(0xFFD8D8D8),
                        fontSize = 11.sp,
                        modifier = Modifier.testTag("trackingStateText")
                    )
                }
            }

            Box(
                modifier = Modifier
                    .fillMaxWidth()
                    .background(Color.Black.copy(alpha = 0.86f))
                    .padding(horizontal = 18.dp, vertical = 10.dp)
            ) {
                Column(
                    modifier = Modifier.fillMaxWidth(),
                    verticalArrangement = Arrangement.spacedBy(8.dp)
                ) {
                    if (settingsExpanded) {
                        Surface(
                            color = Color.White.copy(alpha = 0.08f),
                            shape = RoundedCornerShape(16.dp)
                        ) {
                            Column(
                                modifier = Modifier.padding(12.dp),
                                verticalArrangement = Arrangement.spacedBy(10.dp)
                            ) {
                                Text(
                                    text = "Full pipeline resolution: ${selected.label}",
                                    color = Color.White,
                                    fontSize = 13.sp,
                                    modifier = Modifier.testTag("resolutionPicker")
                                )
                                Text(
                                    text = "(No app-side crop/scale preset)",
                                    color = Color(0xFFCCCCCC),
                                    fontSize = 12.sp
                                )

                                OutlinedButton(
                                    onClick = {
                                        controller.stop()
                                        (context as? Activity)?.finish()
                                    },
                                    modifier = Modifier
                                        .fillMaxWidth()
                                        .height(46.dp)
                                ) {
                                    Text("CLOSE APP")
                                }
                            }
                        }
                    }

                    Box(
                        modifier = Modifier
                            .fillMaxWidth()
                            .height(98.dp),
                        contentAlignment = Alignment.Center
                    ) {
                        Box(
                            modifier = Modifier
                                .align(Alignment.CenterStart)
                                .size(42.dp)
                                .background(Color.White.copy(alpha = 0.10f), CircleShape)
                                .testTag("settingsButton")
                        ) {
                            IconButton(
                                onClick = { settingsExpanded = !settingsExpanded },
                                modifier = Modifier.fillMaxSize()
                            ) {
                                Icon(
                                    imageVector = Icons.Filled.Settings,
                                    contentDescription = "Open settings",
                                    tint = Color.White
                                )
                            }
                        }

                        RecordButton(
                            isRecording = isRecording,
                            onClick = {
                                if (!isRecording) {
                                    if (!hasPermission) {
                                        pendingStart = true
                                        permissionLauncher.launch(requiredPermissions)
                                    } else {
                                        controller.start(selected.size, previewView)
                                    }
                                } else {
                                    controller.stop()
                                }
                            }
                        )
                    }

                    Text(
                        text = when {
                            isRecording -> "Recording • ${formatBytes(recordedBytes)}"
                            recordedBytes > 0 -> "Last recording: ${formatBytes(recordedBytes)}"
                            else -> status
                        },
                        color = Color(0xFFE0E0E0),
                        fontSize = 13.sp,
                        textAlign = TextAlign.Center,
                        modifier = Modifier
                            .fillMaxWidth()
                            .testTag("statusText")
                    )

                    if (settingsExpanded) {
                        datasetPath?.let { path ->
                            Text(
                                text = "Dataset: $path",
                                color = Color(0xFFBDBDBD),
                                fontSize = 11.sp,
                            )
                        }
                    }
                }
            }
        }
    }
}

@Composable
private fun RecordButton(isRecording: Boolean, onClick: () -> Unit) {
    val innerSize = animateDpAsState(
        targetValue = if (isRecording) 32.dp else 62.dp,
        animationSpec = tween(durationMillis = 220, easing = FastOutSlowInEasing),
        label = "record_inner_size"
    )
    val innerCorner = animateDpAsState(
        targetValue = if (isRecording) 9.dp else 31.dp,
        animationSpec = tween(durationMillis = 220, easing = FastOutSlowInEasing),
        label = "record_inner_corner"
    )
    val innerColor = animateColorAsState(
        targetValue = if (isRecording) Color(0xFFE53935) else Color(0xFFF2F2F2),
        animationSpec = tween(durationMillis = 220, easing = FastOutSlowInEasing),
        label = "record_inner_color"
    )
    val outerBorderColor = animateColorAsState(
        targetValue = if (isRecording) Color(0xFF8A2B2B) else Color(0xFF5A5A5A),
        animationSpec = tween(durationMillis = 220, easing = FastOutSlowInEasing),
        label = "record_outer_border"
    )

    Box(
        modifier = Modifier
            .size(92.dp)
            .background(Color(0xFF2B2B2B), CircleShape)
            .border(width = 1.dp, color = outerBorderColor.value, shape = CircleShape)
            .clickable(onClick = onClick)
            .testTag(if (isRecording) "stopButton" else "startButton"),
        contentAlignment = Alignment.Center,
    ) {
        Box(
            modifier = Modifier
                .size(innerSize.value)
                .background(innerColor.value, RoundedCornerShape(innerCorner.value))
        )
    }
}

private fun formatBytes(bytes: Long): String {
    val safeBytes = max(0L, bytes)
    val kb = safeBytes / 1024.0
    val mb = kb / 1024.0
    return if (mb >= 1.0) {
        "%.1f MB".format(mb)
    } else {
        "%.0f KB".format(kb)
    }
}

private fun hasRequiredPermissions(context: android.content.Context, permissions: Array<String>): Boolean {
    val hasCamera = ContextCompat.checkSelfPermission(
        context,
        Manifest.permission.CAMERA
    ) == android.content.pm.PackageManager.PERMISSION_GRANTED

    val hasAnyLocation = permissions
        .filter { it == Manifest.permission.ACCESS_FINE_LOCATION || it == Manifest.permission.ACCESS_COARSE_LOCATION }
        .any {
            ContextCompat.checkSelfPermission(
                context,
                it
            ) == android.content.pm.PackageManager.PERMISSION_GRANTED
        }

    return hasCamera && hasAnyLocation
}

private fun colorForTrackingState(state: String): Color = when (state.uppercase()) {
    "TRACKING" -> Color(0xFF2E7D32)
    "PAUSED", "STARTING", "STARTED" -> Color(0xFFF9A825)
    "STOPPED", "ERROR" -> Color(0xFFC62828)
    else -> Color.Gray
}
