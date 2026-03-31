

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.TeamConstants;

@Autonomous
public class AutoRedfromGoal extends LinearOpMode {
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
        sleep(1555);
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
        scoring[0].setVelocity(1875);
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
        base[0].setPower(0.5);
        base[1].setPower(0.5);
        base[2].setPower(-0.5);
        base[3].setPower(-0.5);
        sleep(260);
        base[0].setPower(0.5);
        base[1].setPower(-0.5);
        base[2].setPower(0.5);
        base[3].setPower(-0.5);
        sleep(520);
        scoring[1].setPower(1);
        scoring[2].setPower(-.65);
        scoring[0].setPower(-1);
        base[0].setPower(0.22);
        base[1].setPower(0.22);
        base[2].setPower(0.22);
        base[3].setPower(0.22);
        sleep(8000);
        scoring[1].setPower(-.3);
        scoring[2].setPower(-.3);
        sleep(150);
        base[0].setPower(-0.5);
        base[1].setPower(-0.5);
        base[2].setPower(-0.5);
        base[3].setPower(-0.5);
        sleep(1000);
        scoring[1].setPower(0);
        base[0].setPower(-1);
        base[1].setPower(1);
        base[2].setPower(-1);
        base[3].setPower(1);
        sleep(580);
        base[0].setPower(-0.5);
        base[1].setPower(-0.5);
        base[2].setPower(0.5);
        base[3].setPower(0.5);
        sleep(235);
        base[0].setPower(0);
        base[1].setPower(0);
        base[2].setPower(0);
        base[3].setPower(0);
        scoring[0].setVelocity(1660);
        sleep(4000);
        scoring[1].setPower(1);
        scoring[2].setPower(1);
        sleep(250);;
        scoring[1].setPower(0);
        scoring[2].setPower(0);
        sleep(3000);
        scoring[1].setPower(1);
        scoring[2].setPower(1);
        sleep(800);
        scoring[0].setPower(0);
        scoring[1].setPower(0);
        scoring[2].setPower(0);
        sleep(230);
        base[0].setPower(0.5);
        base[1].setPower(0.5);
        base[2].setPower(-0.5);
        base[3].setPower(-0.5);
        sleep(80);
        base[0].setPower(-1);
        base[1].setPower(1);
        base[2].setPower(-1);
        base[3].setPower(1);
        sleep(1030);


    }
};