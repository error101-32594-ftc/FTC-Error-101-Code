package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class AutoBasic extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare motors based on config:
        DcMotor frontMotor = hardwareMap.dcMotor.get("frontMotor");
        DcMotor bigHooperMotor = hardwareMap.dcMotor.get("bigHooperMotor");
        DcMotor backMotor = hardwareMap.dcMotor.get("backMotor");
        DcMotor smallHooperMotor = hardwareMap.dcMotor.get("smallHooperMotor");

        frontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        if(isStopRequested()) return;

        while(opModeIsActive()) {
            frontMotor.setPower(1);
            backMotor.setPower(1);
            sleep(2000);
            frontMotor.setPower(0);
            backMotor.setPower(0);
        }
    }
};