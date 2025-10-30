package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class Bad_tele_op extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontMotor = hardwareMap.dcMotor.get("frontMotor");
        DcMotor nothoop = hardwareMap.dcMotor.get("not hooping");
        DcMotor backMotor = hardwareMap.dcMotor.get("backMotor");
        DcMotor HooperMotor = hardwareMap.dcMotor.get("HooperMotor");
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //Bad teleop code for bad no no wheels
            boolean shoot= gamepad1.aWasPressed();
            double leftPower = gamepad1.left_stick_y;
            double rightPower = gamepad1.right_stick_y;

            frontMotor.setPower(-leftPower);
            backMotor.setPower(rightPower);
            if (gamepad1.a){
                nothoop.setPower(1);
                HooperMotor.setPower(-1);
                
            if (gamepad1.bWasPressed());
            {
                HooperMotor.setPower(0);
                frontMotor.setPower(0);
                backMotor.setPower(0);
            }
            }
        }
    }
};
