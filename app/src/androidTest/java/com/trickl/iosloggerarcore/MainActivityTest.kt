package com.trickl.iosloggerarcore

import androidx.compose.ui.test.assertIsDisplayed
import androidx.compose.ui.test.junit4.createAndroidComposeRule
import androidx.compose.ui.test.onNodeWithTag
import org.junit.Rule
import org.junit.Test

class MainActivityTest {
    @get:Rule
    val composeRule = createAndroidComposeRule<MainActivity>()

    @Test
    fun startControlIsVisible() {
        composeRule.onNodeWithTag("resolutionPicker").assertIsDisplayed()
        composeRule.onNodeWithTag("startButton").assertIsDisplayed()
        composeRule.onNodeWithTag("statusText").assertIsDisplayed()
    }
}
