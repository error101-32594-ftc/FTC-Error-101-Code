package org.firstinspires.ftc.teamcode.TeleOp;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.util.TeamConstants;
import org.firstinspires.ftc.teamcode.util.DiagnosticLogger;

import java.io.IOException;
import java.util.Locale;

@TeleOp(group = "Field Centric")
public class Field2Drivers extends LinearOpMode
{
    // Required for the DiagnosticLogger at the bottom of this file:
    private final static IMU.Parameters parameters = TeamConstants.getIMUParms();

    // --- One-time-only constants (non-@Config) ---
    // These only update after a restart of the bot.
    private final static FtcDashboard DASH = FtcDashboard.getInstance();
    private final static TelemetryPacket packet = new TelemetryPacket();

    Telemetry telemetryD = DASH.getTelemetry();
    Telemetry telemetryM = new MultipleTelemetry(telemetry, DASH.getTelemetry());

    @Override
    public void runOpMode()
    {
        // These methods are only initialized when inside of runOpMode:
        telemetryD.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetryM.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        // --- @Config constants ---
        final int CPR = TeamConstants.CPR;
        final double LOCK_ON_DENOMINATOR = TeamConstants.LOCK_ON_DENOMINATOR;
        final double LOCK_ON_OFFSET = TeamConstants.LOCK_ON_OFFSET;
        final double LL_MOUNT_ANGLE = TeamConstants.LL_MOUNT_ANGLE;
        final double LL_LENS_HEIGHT_INCHES = TeamConstants.LL_LENS_HEIGHT_INCHES;
        final double GOAL_HEIGHT_INCHES = TeamConstants.GOAL_HEIGHT_INCHES;

        final DcMotorEx[] base = TeamConstants.getDriveMotors(hardwareMap);
        final DcMotorEx[] scoring = TeamConstants.getScoringMotors(hardwareMap);
        final IMU imu = TeamConstants.getIMU(hardwareMap);
        final Limelight3A limelight = TeamConstants.getLimelight(hardwareMap);

        // Final setup before control starts:
        imu.initialize(parameters);
        limelight.start();

        telemetryM.addLine("> Ready.");
        telemetryM.update();

        waitForStart();

        // Ensure telemetry is clear on start.
        // FtcDashboard tends to keep telemetry in the buffer even after
        // update()-ing.
        // (We abuse this a bit later to have nicely formatted text, since
        // FtcDashboard also likes alphabetizing data.)
        telemetryM.clear();

        DiagnosticLogger logger = getLogger();
        Thread loggerRuntimeThread = new Thread(logger);
        logger.resumeRun();
        loggerRuntimeThread.start();

        boolean lockOn = false;
        boolean autoShot = true;
        boolean pastLeftBumper1 = false;
        boolean pastB2 = false;
        int meanCounter = 0;
        double aggregateDistance = 0;
        double meanDistance = 0;

        while(opModeIsActive())
        {
            LLResult rawResult = limelight.getLatestResult();
            boolean resultIsValid = rawResult.isValid();

            // LockOn logic
            double rx;
            if(lockOn && resultIsValid)
            {
                double targetX = rawResult.getTx();
                // A simple proportional input; May use PI (proportional-integral)
                // control in the future.
                // getTx returns a value in the range of -27.25 to 27.25.
                // We want to get a value from -1 to 1, so we can just divide
                // by 27.25. (27.25/27.25 = 1)
                rx = (targetX+LOCK_ON_OFFSET)/LOCK_ON_DENOMINATOR;
            } else
            {
                rx = gamepad1.right_stick_x;
            }

            // "Artemis Take the Flywheel" logic
            double hooperPower, rawDistance = 0;
            if(autoShot)
            {
                double targetY = rawResult.getTy();
                rawDistance =
                        (GOAL_HEIGHT_INCHES - LL_LENS_HEIGHT_INCHES) / Math.tan(
                            ((LL_MOUNT_ANGLE + targetY) * (Math.PI/180.0))
                        )
                ;

                //if (meanCounter < 10) {
                //    aggregateDistance += rawDistance;
                //    meanCounter++;
                //}
                //else
                //{
                //    oldMean = aggregateDistance / 10;
                //
                //    aggregateDistance -= oldMean;
                //
                //    aggregateDistance += rawDistance;
                //
                //    meanDistance = aggregateDistance / 10;
                //}

                aggregateDistance += rawDistance;
                meanCounter++;

                if(meanCounter > 10)
                {
                    meanDistance = aggregateDistance / 10;

                    meanCounter = 0;
                    aggregateDistance = 0;
                }

                if(rawDistance >= 65.7)
                {
                    hooperPower = (28.6*rawDistance)+2623;
                } else if(rawDistance >= 47.5)
                {
                    hooperPower = (16.5*rawDistance)+3416;
                } else if(rawDistance >= 38.8)
                {
                    hooperPower = (8.62*rawDistance)+3791;
                } else
                {
                    hooperPower = 3000;
                }
                meanDistance = 0;
            } else
            {
                // Max target RPM: 4900
                hooperPower = 3000 + (gamepad2.left_trigger * 1900);
            }
            double driveBreakPower = 1-gamepad1.right_trigger;

            // Gamepad variables:
            boolean a1 = gamepad1.a;
            boolean a2 = gamepad2.a;
            boolean b1 = gamepad1.b;
            boolean b2 = gamepad2.b;
            boolean x1 = gamepad1.x;
            boolean x2 = gamepad2.x;
            boolean y1 = gamepad1.y;
            boolean y2 = gamepad2.y;
            boolean leftBumper1 = gamepad1.left_bumper;
            boolean rightBumper2 = gamepad2.right_bumper;
            double rightTrigger2 = gamepad2.right_trigger;
            boolean start1 = gamepad1.start;

            double lsY1 = -gamepad1.left_stick_y;
            double lsX1 = gamepad1.left_stick_x*1.1;


            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double botHeadingDegrees = botHeading * (180.0/Math.PI);
            // Rotate the movement direction counter to the bot's rotation
            double rotX = 1.1*(
                    lsX1 * Math.cos(-botHeading) - lsY1 * Math.sin(-botHeading)
            );
            double rotY =
                    lsX1 * Math.sin(-botHeading) + lsY1 * Math.cos(-botHeading)
            ;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(
                    Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1
            );
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            // -- Runtime Telemetry --
            double hooperSpeed = (scoring[0].getVelocity()*60)/CPR;
            String runtimeSeconds = String.format(
                    Locale.ENGLISH, "%.4f", getRuntime()
            );

            packet.clearLines();
            packet.put("<span style=display:none;>",
                    "</span>" +
                            "Runtime (Seconds): " + runtimeSeconds + "<br>" +
                            "<br>" +
                            "== Shooting Motor (scoring[0]) =="
            );

            // runtimeSeconds shouldn't be graphable,so we make it a line
            // instead of data. (FtcDashboard graph already adds time as X)
            DASH.sendTelemetryPacket(packet);
            telemetryD.addData("shooting-input-rpm", hooperPower);
            telemetryD.addData("shooting-speed-rpm", hooperSpeed);
            telemetryD.update();
            telemetryD.addData("<br><span style=display:none;>", "</span>");
            telemetryD.update();
            telemetryD.addData("right-stick-x", rx);
            telemetryD.addData("bot-heading", botHeadingDegrees);
            telemetryD.addData("auto-shot", autoShot);
            telemetryD.addData("ll-distance", rawDistance);
            telemetryD.addData("lock-on", lockOn);
            telemetryD.addData("valid-target", resultIsValid);
            telemetryD.update();

            telemetry.addData("Runtime (Seconds)", runtimeSeconds);
            telemetry.addLine();
            telemetry.addLine("== Shooting Motor (scoring[0]) ==");
            telemetry.addData("shooting-input-rpm", hooperPower);
            telemetry.addData("shooting-speed-rpm", hooperSpeed);
            telemetry.addLine();
            telemetry.addData("right-stick-x", rx);
            telemetry.addData("bot-heading", botHeadingDegrees);
            telemetry.addData("auto-shot", autoShot);
            telemetry.addData("ll-distance", rawDistance);
            telemetry.addData("lock-on", lockOn);
            telemetry.addData("valid-target", resultIsValid);
            telemetry.update();

            if (a1 && b1 && x1 && y1)
            {
                logger.stopRun();
            }

            if (start1)
            {
                imu.resetYaw();
            }

            // LockOn toggle
            if(leftBumper1 && ! pastLeftBumper1)
            {
                lockOn = !lockOn;

            }

            if(y2)
            {
                scoring[0].setPower(-0.9);
            } else
            {
                scoring[0].setVelocity((hooperPower/60) * CPR);
            }

            if (rightBumper2 && a2)
            {
                scoring[1].setPower(-0.9);
            }  else if (a2)
            {
                scoring[1].setPower(0.9);
            } else
            {
                scoring[1].setPower(0);
            }
            if(rightBumper2 && x2)
            {
                scoring[2].setPower(-0.9);
            } else if(x2)
            {
                scoring[2].setPower(0.9);
            } else
            {
                scoring[2].setPower(0);
            }

            // "Artemis Take the Flywheel" toggle
            if(rightTrigger2 == 1 && b2 && ! pastB2)
            {
                autoShot = !autoShot;
            }

            base[0].setPower(frontLeftPower*driveBreakPower);
            base[1].setPower(backLeftPower*driveBreakPower);
            base[2].setPower(backRightPower*driveBreakPower);
            base[3].setPower(frontRightPower*driveBreakPower);

            pastLeftBumper1 = leftBumper1;
            pastB2 = b2;
        }
    }

    @NonNull
    private DiagnosticLogger getLogger()
    {
        DiagnosticLogger logger;
        try
        {
            logger = new DiagnosticLogger(
                    telemetry, hardwareMap,
                    null,
                    new String[]
                    {
                            "fl", "rl",
                            "fr", "rr",
                            "hooper", "intake1", "intake2"
                    },
                    null, "imu", parameters
            );
        } catch (IOException e)
        {
            throw new RuntimeException(e);
        }
        return logger;
    }
}
