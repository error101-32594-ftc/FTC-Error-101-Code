package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.TeamConstants;

@Autonomous
public class AutoBasic extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare motors
        // Make sure your ID's match your configuration
        final DcMotorEx[] base = TeamConstants.getDriveMotors(hardwareMap);
        final DcMotorEx[] scoring = TeamConstants.getScoringMotors(hardwareMap);

        waitForStart();

        if(isStopRequested()) return;

        base[0].setPower(-0.5);
        base[1].setPower(-0.5);
        base[2].setPower(-0.5);
        base[3].setPower(-0.5);
        sleep(1000);
        /*base[0].setPower(0.5);
        base[1].setPower(0.5);
        base[2].setPower(0.5);
        base[3].setPower(0.5);
        base[0].setPower(0.25);
        base[1].setPower(0.25);
        base[2].setPower(-0.25);
        base[3].setPower(-0.25); */
        //sleep(750);
        base[0].setPower(0);
        base[0].setPower(0);
        base[0].setPower(0);
        base[0].setPower(0);
        scoring[0].setVelocity(4500);
        sleep(2000);
        scoring[1].setPower(1);
        scoring[2].setPower(1);
        sleep(600);
        scoring[1].setPower(0);
        scoring[2].setPower(0);
        sleep(500);
        scoring[1].setPower(1);
        scoring[2].setPower(1);
        sleep(800);
        scoring[0].setPower(0);
        scoring[1].setPower(0);
        scoring[2].setPower(0);


    }
};
