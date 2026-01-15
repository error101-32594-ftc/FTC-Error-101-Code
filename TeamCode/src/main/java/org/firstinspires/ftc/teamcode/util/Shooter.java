package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(group = "Testing")
public class Shooter extends LinearOpMode
{
    private final int CPR = TeamConstants.CPR;
    private final double LL_MOUNT_ANGLE = TeamConstants.LL_MOUNT_ANGLE;
    private final double LL_LENS_HEIGHT_INCHES = TeamConstants.LL_LENS_HEIGHT_INCHES;
    private final double GOAL_HEIGHT_INCHES = TeamConstants.GOAL_HEIGHT_INCHES;

    private final static FtcDashboard DASH = FtcDashboard.getInstance();

    @Override
    public void runOpMode()
    {
        // CONSTANTS:
        int INCREMENT = 25;
        //

        Telemetry telemetryM = new MultipleTelemetry(
                telemetry, DASH.getTelemetry()
        );
        telemetryM.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        DcMotorEx[] scoring = TeamConstants.getScoringMotors(hardwareMap);
        DcMotorEx[] base = TeamConstants.getDriveMotors(hardwareMap);
        IMU.Parameters parameters = TeamConstants.getIMUParms();
        IMU imu = TeamConstants.getIMU(hardwareMap);

        Limelight3A limelight = TeamConstants.getLimelight(hardwareMap);

        imu.initialize(parameters);
        limelight.start();

        telemetryM.addLine("Ready.");
        telemetryM.update();

        waitForStart();

        telemetryM.clear();

        // Variables for run loop:
        double shootingInputRPM = 0;
        boolean lastAccelerate = false;
        boolean lastDecelerate = false;
        //

        while(opModeIsActive())
        {
            // API Variables:
            boolean accelerate = gamepad1.y;
            boolean decelerate = gamepad1.a;
            boolean intake = gamepad1.b;
            boolean intakeReverse = gamepad1.x;

            LLResult result = limelight.getLatestResult();
            double targetY = result.getTy();
            //

            if((accelerate && decelerate) && !(lastAccelerate && lastDecelerate))
            {
                shootingInputRPM = 0;
            } else if(accelerate && ! lastAccelerate)
            {
                shootingInputRPM += INCREMENT;
            } else if(decelerate && ! lastDecelerate)
            {
                shootingInputRPM -= INCREMENT;
            }

            if(intake)
            {
                scoring[1].setPower(0.9);
                scoring[2].setPower(0.9);
            } else if(intakeReverse)
            {
                scoring[1].setPower(-0.9);
                scoring[2].setPower(-0.9);
            } else
            {
                scoring[1].setPower(0);
                scoring[2].setPower(0);
            }

            double lsY1 = -gamepad1.left_stick_y;
            double lsX1 = gamepad1.left_stick_x*1.1;
            double rx = gamepad1.right_stick_x;


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
            double backRightPower = (rotY - rotX - rx) / denominator;
            double frontRightPower = (rotY + rotX - rx) / denominator;


            scoring[0].setVelocity((shootingInputRPM / 60) * CPR);
            base[0].setPower(frontLeftPower);
            base[1].setPower(backLeftPower);
            base[2].setPower(frontRightPower);
            base[3].setPower(backRightPower);

            telemetryM.addData("shooting-input-rpm", shootingInputRPM);
            telemetryM.addData("shooting-speed-rpm", (scoring[0].getVelocity() * 60) / CPR);
            telemetryM.addData("distance-to-target",
                    (GOAL_HEIGHT_INCHES - LL_LENS_HEIGHT_INCHES) / Math.tan(
                            ((LL_MOUNT_ANGLE + targetY) * (Math.PI/180.0))
                    )
            );
            telemetryM.update();

            lastAccelerate = accelerate;
            lastDecelerate = decelerate;
        }
    }
}
