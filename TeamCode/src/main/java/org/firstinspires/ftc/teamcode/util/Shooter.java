package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(group = "Testing")
public class Shooter extends LinearOpMode
{
    private final int CPR = TeamConstants.CPR;

    private final static FtcDashboard DASH = FtcDashboard.getInstance();

    @Override
    public void runOpMode()
    {
        // CONSTANTS:
        int INCREMENT = 25;
        double LL_MOUNT_ANGLE = 30;
        double LL_LENS_HEIGHT_INCHES = 14.5;
        double GOAL_HEIGHT_INCHES = 38.75;
        //

        Telemetry telemetryM = new MultipleTelemetry(
                telemetry, DASH.getTelemetry()
        );
        telemetryM.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        DcMotorEx[] scoring = TeamConstants.getScoringMotors(hardwareMap);

        Limelight3A limelight = TeamConstants.getLimelight(hardwareMap);

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
            } else if(intakeReverse)
            {
                scoring[1].setPower(-0.9);
            } else
            {
                scoring[1].setPower(0);

            }

            scoring[0].setVelocity((shootingInputRPM / 60) * CPR);

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
