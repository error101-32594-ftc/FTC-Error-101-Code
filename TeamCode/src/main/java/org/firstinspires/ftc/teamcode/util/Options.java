package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group = "Management")
public class Options extends LinearOpMode
{
    private final String[] prompts = {
            "Begin?",
            "What alliance are you playing for?",
            "Where do you plan to start from? (North is where the goals are.)",
            "How many drivers?",
            "Preferred driving style?"
    };

    private String[] modes = {"", "", "", "", "", ""};
    private final boolean[] inputs = {
            gamepad1.a, gamepad1.b, gamepad1.x, gamepad1.y,
            gamepad1.left_bumper, gamepad1.right_bumper
    };

    @Override
    public void runOpMode() throws InterruptedException
    {
        String currentPrompt = prompts[0];

        telemetry.addData("Prompt", currentPrompt);
        telemetry.addData(" A", modes[0]);
        telemetry.addData(" B", modes[1]);
        telemetry.addData(" X", modes[2]);
        telemetry.addData(" Y", modes[3]);
        telemetry.addData("LB", modes[4]);
        telemetry.addData("RB", modes[5]);

        for(String prompt : prompts)
        {
            System.out.println(prompt);
        }
    }
}
