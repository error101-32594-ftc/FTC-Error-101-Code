package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.Encoder;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;

import com.pedropathing.paths.PathConstraints;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Constants {
    public static double multiplierForward = 0.08294676287147268;
    public static double multiplierStrafe = 0.008038662919454933;
    public static double multiplierTurn = 0.02731472448271;


    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(8.9);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName("fl")
            .leftRearMotorName("rl")
            .rightRearMotorName("rr")
            .rightFrontMotorName("fr")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD);
    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            .leftFrontMotorName("fl")
            .leftRearMotorName("rl")
            .rightRearMotorName("rr")
            .rightFrontMotorName("fr")
            .robotLength(17.6)
            .forwardTicksToInches(multiplierForward)
            .strafeTicksToInches(multiplierStrafe)
            .turnTicksToInches(multiplierTurn)
            .robotWidth(16.6)
            .leftFrontEncoderDirection(Encoder.FORWARD)
            .leftRearEncoderDirection(Encoder.FORWARD)
            .rightRearEncoderDirection(Encoder.REVERSE)
            .rightFrontEncoderDirection(Encoder.FORWARD);
    /**
     These are the PathConstraints in order:
     tValueConstraint, velocityConstraint, translationalConstraint, headingConstraint, timeoutConstraint,
     brakingStrength, BEZIER_CURVE_SEARCH_LIMIT, brakingStart

     The BEZIER_CURVE_SEARCH_LIMIT should typically be left at 10 and shouldn't be changed.
     */

    public static PathConstraints pathConstraints = new PathConstraints(        1.0,    // tValueConstraint
            50.0,   // velocityConstraint
            50.0,   // translationalConstraint
            Math.toRadians(180), // headingConstraint
            3.0,    // timeoutConstraint
            1.0,    // brakingStrength
            10,     // BEZIER_CURVE_SEARCH_LIMIT (leave at 10)
            0.3     // brakingStart
            );

    //Add custom localizers or drivetrains here
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .driveEncoderLocalizer(localizerConstants)
                .build();
    }
}