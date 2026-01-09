package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.paths.PathConstraints;

@Config
public class Constants
{
    // Counts-Per-Revolution of a REV HD Hex Motor.
    public static int CPR = 28;
    public static double LOCK_ON_DENOMINATOR = 27.25;

    public static DcMotorEx[] getDriveMotors(HardwareMap hardwareMap)
    {
            // --- Get Motor Objects ---
            DcMotorEx[] motors = {
                    hardwareMap.get(DcMotorEx.class, "frontLeftMotor"),
                    hardwareMap.get(DcMotorEx.class, "backLeftMotor"),
                    hardwareMap.get(DcMotorEx.class, "backRightMotor"),
                    hardwareMap.get(DcMotorEx.class, "frontRightMotor")

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
                hardwareMap.get(DcMotorEx.class, "bigHooperMotor"),
                hardwareMap.get(DcMotorEx.class, "smallHooperMotor")
        };

        // --- Motor Placement ---
        // "F": Front; "B": Back
        // + --F-- +
        // | 1     |
        // |  0    |
        // + --B-- +

        // --- Set Motor Directions ---
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1].setDirection(DcMotorSimple.Direction.FORWARD);

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

    // --- PedroPathing Constants ---
    public static FollowerConstants followerConstants = new FollowerConstants();

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99, 100, 1, 1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .build();
    }
}
