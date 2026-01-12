package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

@Config
public class TeamConstants
{
    // Counts-Per-Revolution of a REV HD Hex Motor.
    public static int CPR = 28;
    public static double LOCK_ON_DENOMINATOR = 27.25;
    public static double LOCK_ON_OFFSET = 0.0;

    public static DcMotorEx[] getDriveMotors(HardwareMap hardwareMap)
    {
            // --- Get Motor Objects ---
            DcMotorEx[] motors = {
                    hardwareMap.get(DcMotorEx.class, "fl"),
                    hardwareMap.get(DcMotorEx.class, "rl"),
                    hardwareMap.get(DcMotorEx.class, "rr"),
                    hardwareMap.get(DcMotorEx.class, "fr")

            };

            // --- Motor Placement ---
            // "F": Front; "B": Back
            //   0 --F-- 3
            //   |       |
            //   |       |
            //   1 --B-- 2

            // --- Set Motor Directions ---
            motors[0].setDirection(DcMotorSimple.Direction.FORWARD);
            motors[1].setDirection(DcMotorSimple.Direction.FORWARD);
            motors[2].setDirection(DcMotorSimple.Direction.REVERSE);
            motors[3].setDirection(DcMotorSimple.Direction.FORWARD);

            // --- Set Motor ZPB ---
            for(DcMotorEx motor : motors)
            {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

        return motors;
    }

    public static DcMotorEx[] getScoringMotors(HardwareMap hardwareMap)
    {
        // --- Get Motor Objects ---
        DcMotorEx[] motors = {
                hardwareMap.get(DcMotorEx.class, "hooper"),
                hardwareMap.get(DcMotorEx.class, "intake1"),
                hardwareMap.get(DcMotorEx.class, "intake2")
        };

        // --- Motor Placement ---
        // "F": Front; "B": Back
        // + --F-- +
        // | 1     |
        // | 20    |
        // + --B-- +

        // --- Set Motor Directions ---
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[2].setDirection(DcMotorSimple.Direction.REVERSE);

        return motors;
    }

    public static IMU.Parameters getIMUParms()
    {
        return new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
    }

    public static IMU getIMU(HardwareMap hardwareMap)
    {
        return hardwareMap.get(IMU.class, "imu");
    }

    public static Limelight3A getLimelight(HardwareMap hardwareMap)
    {
        Limelight3A limelight3a = hardwareMap.get(
                Limelight3A.class, "limelight3a"
        );
        limelight3a.setPollRateHz(100);

        return limelight3a;
    }


}
