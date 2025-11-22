package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.io.IOException;

@TeleOp(name = "Mecanum - 1 Driver", group = "Robot Centric")
public class TeleOpMecanumRobot1Driver extends LinearOpMode {
    private IMU.Parameters parameters;
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor bigHooperMotor = hardwareMap.dcMotor.get("bigHooperMotor");
        DcMotor smallHooperMotor = hardwareMap.dcMotor.get("smallHooperMotor");

        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // The private parameters variable is passed to the logger at the bottom of this class.
        this.parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo UP / USB FORWARD
        imu.initialize(parameters);

        DiagnosticLogger logger = getLogger();
        Thread loggerRuntimeThread = new Thread(logger);
        logger.resumeRun();
        loggerRuntimeThread.start();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;
            double brakePower = 1 - gamepad1.right_trigger;
            double bigHooperPower = -gamepad1.left_trigger;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            telemetry.addData("Right Stick X", rx);
            telemetry.addData("Left Stick X", x);
            telemetry.addData("Left Stick Y", y);
            telemetry.addData("Right Trigger", gamepad1.right_trigger);
            telemetry.addData("Left Trigger", gamepad1.left_trigger);
            telemetry.addData("frontLeftMotor", frontLeftMotor.getPower());
            telemetry.addData("frontRightMotor", frontRightMotor.getPower());
            telemetry.addData("backLeftMotor", backLeftMotor.getPower());
            telemetry.addData("backRightMotor", backRightMotor.getPower());
            telemetry.addData("Voltage", voltageSensor.getVoltage());
            telemetry.addData("A", gamepad1.a);
            telemetry.addData("B", gamepad1.b);
            telemetry.addData("X", gamepad1.x);
            telemetry.addData("Y", gamepad1.y);
            telemetry.update();

            if (gamepad1.a && gamepad1.b && gamepad1.x && gamepad1.y)
            {
                logger.stopRun();
            }

            if (gamepad1.a)
            {
                smallHooperMotor.setPower(0.9);
                bigHooperMotor.setPower(bigHooperPower);
            } else
            {
                smallHooperMotor.setPower(0);
                bigHooperMotor.setPower(0);
            }

            if (gamepad1.b)
            {
                bigHooperMotor.setPower(0);
            }

            frontLeftMotor.setPower(frontLeftPower * brakePower);
            backLeftMotor.setPower(backLeftPower * brakePower);
            frontRightMotor.setPower(frontRightPower * brakePower);
            backRightMotor.setPower(backRightPower * brakePower);
        }
    }

    @NonNull
    private DiagnosticLogger getLogger()
    {
        DiagnosticLogger logger;
        try
        {
            logger = new DiagnosticLogger(
                    telemetry, hardwareMap,
                    new String[]
                            {
                                    "frontLeftMotor", "backLeftMotor",
                                    "frontRightMotor", "backRightMotor",
                                    "bigHooperMotor", "smallHooperMotor"
                            },
                    null, null, "imu", parameters
            );
        } catch (IOException e)
        {
            telemetry.addData("IOException", e.getMessage());
            throw new RuntimeException(e);
        }
        return logger;
}
}
