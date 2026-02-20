package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class ServoPsuedoEncoderTest extends LinearOpMode {

    private CRServo crServo;

    @Override
    public void runOpMode() {

        // Get the CR servo from the hardware map
        crServo = hardwareMap.get(CRServo.class, "cr_servo");

        // Optionally set the direction
        //crServo.setDirection(CRServo.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Drive with right stick Y (gamepad1)
            int Degrees = 1;
            double Servo_degree_time =  1.13666666;
            double Target_time = Degrees*Servo_degree_time;
            crServo.setPower(0.9);
            try {
                Thread.sleep((long) Target_time);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            crServo.setpower(0);

            telemetry.addData("CR Servo Power", power);
            telemetry.update();
        }

        // Stop the servo when the op mode ends
        crServo.setPower(0);
    }
}