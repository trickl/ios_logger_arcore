@ECHO OFF
SETLOCAL

SET DIRNAME=%~dp0
IF "%DIRNAME%"=="" SET DIRNAME=.
SET APP_HOME=%DIRNAME%
SET CLASSPATH=%APP_HOME%gradle\wrapper\gradle-wrapper.jar

IF EXIST "%CLASSPATH%" GOTO run
ECHO ERROR: Missing %CLASSPATH%
ECHO Run the Gradle 'wrapper' task from Android Studio, or add gradle-wrapper.jar to gradle/wrapper/.
EXIT /B 1

:run
IF NOT "%JAVA_HOME%"=="" IF EXIST "%JAVA_HOME%\bin\java.exe" (
  SET JAVACMD=%JAVA_HOME%\bin\java.exe
) ELSE (
  SET JAVACMD=java.exe
)

"%JAVACMD%" -classpath "%CLASSPATH%" org.gradle.wrapper.GradleWrapperMain %*
ENDLOCAL
