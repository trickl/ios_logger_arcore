#!/bin/sh

# Minimal Gradle wrapper launcher.
# Delegates to gradle/wrapper/gradle-wrapper.jar, which downloads the configured Gradle distribution.

APP_HOME=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
CLASSPATH="$APP_HOME/gradle/wrapper/gradle-wrapper.jar"

if [ ! -f "$CLASSPATH" ]; then
  echo "ERROR: Missing $CLASSPATH"
  echo "Run the Gradle 'wrapper' task from Android Studio, or add gradle-wrapper.jar to gradle/wrapper/."
  exit 1
fi

if [ -n "$JAVA_HOME" ] && [ -x "$JAVA_HOME/bin/java" ]; then
  JAVACMD="$JAVA_HOME/bin/java"
else
  JAVACMD="java"
fi

exec "$JAVACMD" -classpath "$CLASSPATH" org.gradle.wrapper.GradleWrapperMain "$@"
