package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.TeamConstants;

@Autonomous
public class AutoForward extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        final DcMotorEx[] base = TeamConstants.getDriveMotors(hardwareMap);
        final DcMotorEx[] scoring = TeamConstants.getScoringMotors(hardwareMap);

        base[0].setPower(0.5);
        base[1].setPower(0.5);
        base[2].setPower(0.5);
        base[3].setPower(0.5);
        sleep(750);
        base[0].setPower(0);
        base[1].setPower(0);
        base[2].setPower(0);
        base[3].setPower(0);
    }
}
