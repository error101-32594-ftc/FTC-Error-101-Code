package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class AutoBasic extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare motors
        // Make sure your ID's match your configuration
        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        DcMotorEx bigHooperMotor = hardwareMap.get(DcMotorEx.class, "bigHooperMotor");
        DcMotorEx smallHooperMotor = hardwareMap.get(DcMotorEx.class, "smallHooperMotor");

        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if(isStopRequested()) return;

        frontLeftMotor.setPower(0.5);
        frontRightMotor.setPower(0.5);
        backLeftMotor.setPower(0.5);
        backRightMotor.setPower(0.5);
        sleep(2000);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
};