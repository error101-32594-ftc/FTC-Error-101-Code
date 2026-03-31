

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.TeamConstants;

@Autonomous
public class AutoBlueBack extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare motors
        // Make sure your ID's match your configuration
        final DcMotorEx[] base = TeamConstants.getDriveMotors(hardwareMap);
        final DcMotorEx[] scoring = TeamConstants.getScoringMotors(hardwareMap);

        waitForStart();

        if(isStopRequested()) return;


        base[0].setPower(-1);
        base[1].setPower(1);
        base[2].setPower(-1);
        base[3].setPower(1);
        sleep(430);


    }
};