# Error 101 #32594

## DiagnosticsLogger (/TeamCode/src/main/java/.../DiagnosticsLogger.java)

DiagnosticsLogger is a Runnable class our team made to be able to easily gather
data from our robot, focusing on the motors. It is mostly based off of the
datalogger provided by the *FIRST Tech Challenge* GitHub organization's
WikiSupport repository, but ours is designed to run in the background during any
OpMode instead of on-demand logging like the WikiSupport repo version provides.

### Initialization:

Parameters:

1. Telemetry:      telemetry

2. HardwareMap:    hardwareMap

3. String[]:       DcMotor Names

4. String[]:       DcMotorEx Names

5. String[]:       CrServo Names

6. String:         IMU Name

7. IMU.Parameters: IMU Parameters

Suggested Method:

```java
private DiagnosticLogger getLogger()
{
      DiagnosticLogger logger;
      try
      {
            logger = new DiagnosticLogger(
                  // Replace with the names of your telemetry and hardwareMap
                  // instances, if they are customized:
                  telemetry, hardwareMap,
                  new String[] {
                        // Include names of DcMotors...
                  },
                  new String[] {
                        // Include names of DcMotorExs...
                  },
                  new String[] {
                        // Include names of CrServos...
                  },
                  // Replace with the name of your IMU device and parameter
                  // variable, if they are customized:
                  "imu", parameters

                  // Any of the parameters can be replaced with null if they
                  // aren't relevant for your robot.
            );
      } catch (IOException e)
      {
            telemetry.addData("IOException", e.getMessage());
            throw new RuntimeException(e);
      }
      return logger;
}
```

### Usage:

You'll probably want to make a new Thread from the Runnable object returned by
the above method, so you can have it run in the background:

```java
DiagnosticLogger logger = getLogger();
Thread loggerRuntimeThread = new Thread(logger);
```

You can now use the Runnable object to resume, pause, and stop the logger using
the `resumeRun()`, `pauseRun()`, and `stopRun()` methods included.

The logger is in a paused state by default, so you'll have to run:

```java
logger.resumeRun();
loggerRuntimeThread.start();
```

to start it instead of just `loggerRuntimeThread.start();`.
