

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.TeamConstants;

@Autonomous
public class AutoBluefromGoal extends LinearOpMode {
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
        sleep(1550);
        base[0].setPower(0.5);
        base[1].setPower(0.5);
        base[2].setPower(-0.5);
        base[3].setPower(-0.5);
        sleep(40);
        /*base[0].setPower(0.5);
        base[1].setPower(0.5);
        base[2].setPower(0.5);
        base[3].setPower(0.5); */
        base[0].setPower(0);
        base[1].setPower(0);
        base[2].setPower(0);
        base[3].setPower(0);
        scoring[0].setVelocity(1855);
        sleep(4000);
        scoring[1].setPower(0);
        scoring[2].setPower(1);
        sleep(250);;
        scoring[1].setPower(0);
        scoring[2].setPower(0);
        sleep(2500);
        scoring[1].setPower(1);
        scoring[2].setPower(1);
        sleep(650);
        scoring[0].setPower(0);
        scoring[1].setPower(0);
        scoring[2].setPower(0);
        base[0].setPower(-0.5);
        base[1].setPower(-0.5);
        base[2].setPower(0.5);
        base[3].setPower(0.5);
        sleep(327);
        base[0].setPower(-0.5);
        base[1].setPower(0.5);
        base[2].setPower(-0.5);
        base[3].setPower(0.5);
        sleep(638);
        scoring[1].setPower(1);
        scoring[2].setPower(-.70);
        scoring[0].setPower(-1);
        base[0].setPower(0.22);
        base[1].setPower(0.22);
        base[2].setPower(0.22);
        base[3].setPower(0.22);
        sleep(5950);
        scoring[1].setPower(-.35);
        scoring[2].setPower(-.35);
        sleep(100);
        scoring[1].setPower(0);
        scoring[2].setPower(0);
        base[0].setPower(-0.5);
        base[1].setPower(-0.5);
       base[2].setPower(-0.5);
        base[3].setPower(-0.5);
        sleep(1000);
        scoring[1].setPower(0);
        base[0].setPower(1);
        base[1].setPower(-1);
        base[2].setPower(1);
        base[3].setPower(-1);
        sleep(580);
        base[0].setPower(0.5);
        base[1].setPower(0.5);
        base[2].setPower(-0.5);
        base[3].setPower(-0.5);
        sleep(310);
        base[0].setPower(0);
        base[1].setPower(0);
        base[2].setPower(0);
        base[3].setPower(0);
        scoring[0].setVelocity(1745);
        sleep(4000);
        scoring[1].setPower(1);
        scoring[2].setPower(1);
        sleep(250);;
        scoring[1].setPower(0);
        scoring[2].setPower(0);
        sleep(3300);
        scoring[1].setPower(1);
        scoring[2].setPower(1);
        sleep(800);
        scoring[0].setPower(0);
        scoring[1].setPower(0);
        scoring[2].setPower(0);
        sleep(230);
        base[0].setPower(-0.5);
        base[1].setPower(-0.5);
        base[2].setPower(0.5);
        base[3].setPower(0.5);
        sleep(70);
        base[0].setPower(1);
        base[1].setPower(-1);
        base[2].setPower(1);
        base[3].setPower(-1);
        sleep(1030);


    }
};