package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="SimpleAuto_ForwardTurnMotor", group="Robot")
public class SimpleAuto extends LinearOpMode {

    // Declare hardware variables
    private DcMotor leftFront, rightFront, leftBack, rightBack, auxMotor;

    @Override
    public void runOpMode() {
        // 1. Initialize hardware (names must match your Robot Configuration)
        leftFront  = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftBack   = hardwareMap.get(DcMotor.class, "left_back");
        rightBack  = hardwareMap.get(DcMotor.class, "right_back");
        auxMotor   = hardwareMap.get(DcMotor.class, "aux_motor");

        // Reverse the left side so positive power moves forward
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive()) {

            // STEP 1: Move Forward (Power, Time in Milliseconds)
            drive(0.5, 0.5, 1000);

            // STEP 2: Turn 180 Degrees
            // (Turning requires left motors and right motors to move in opposite directions)
            drive(0.5, -0.5, 1200); // Adjust time based on your robot's friction/speed

            // STEP 3: Run Auxiliary Motor
            auxMotor.setPower(0.8);
            sleep(2000); // Run for 2 seconds
            auxMotor.setPower(0);

            telemetry.addData("Status", "Complete");
            telemetry.update();
        }
    }

    /**
     * Helper method to set motor powers and sleep
     */
    public void drive(double leftPower, double rightPower, int duration) {
        leftFront.setPower(leftPower);
        leftBack.setPower(leftPower);
        rightFront.setPower(rightPower);
        rightBack.setPower(rightPower);

        sleep(duration);

        // Stop all motors
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}