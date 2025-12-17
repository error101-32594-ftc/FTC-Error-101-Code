package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TeleOpBasic extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontMotor = hardwareMap.dcMotor.get("fm");
        DcMotor leftmomtor = hardwareMap.dcMotor.get("mf");
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double fmp;
            double uhidk;
            fmp=gamepad1.right_trigger;
            uhidk= -fmp;

            frontMotor.setPower(fmp);
            leftmomtor.setPower(uhidk);
            }
        }
    }
