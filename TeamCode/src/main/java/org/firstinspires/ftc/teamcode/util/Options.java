package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

@Autonomous(group = "Management")
public class Options extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        // Just asking about alliance for now.
        String prompt = "What alliance are you playing for?";

        telemetry.addLine(prompt);
        telemetry.addLine("For RED Alliance, press B.");
        telemetry.addLine("For BLU Alliance, press X.");
        telemetry.addLine("The screen will clear when the OpMode starts.");
        telemetry.update();

        waitForStart();

        while(opModeIsActive())
        {
            boolean red = gamepad1.b;
            boolean blu = gamepad1.x;
            boolean lb = gamepad1.left_bumper;

            String select = "";
            if(red)
            {
                select = "RED";
            } else if(blu)
            {
                select = "BLU";
            }

            telemetry.clear();

            if(lb)
            {
                telemetry.addLine(select + " was selected.");
                telemetry.addLine("");
                telemetry.update();

                // Writing file:
                String filename = "/sdcard/FIRST/DiagnosticLogs/alliance.txt";
                String alliance = select.equals("BLU") ? "1" : "0";
                try
                {
                    AllianceWriter writer = new AllianceWriter(filename);
                    writer.write(alliance);
                } catch(IOException e)
                {
                    throw new RuntimeException(e);
                }
            } else
            {
                telemetry.addLine(select + " is currently selected.");
                telemetry.addLine("Press LEFT_BUMPER to finalize selection");
                telemetry.update();
            }
        }
    }

    private static class AllianceWriter
    {
        final FileWriter fileWriter;
        private final BufferedWriter bufferedWriter;

        public AllianceWriter(String filename) throws IOException
        {
            File tmp = new File(filename);
            if(!tmp.exists())
            {
                tmp.getParentFile().mkdirs();
            }
            fileWriter = new FileWriter(filename, false);
            bufferedWriter = new BufferedWriter(fileWriter);
        }

        public void write(String alliance) throws IOException
        {
            bufferedWriter.write(alliance);
            bufferedWriter.flush();
            bufferedWriter.close();
        }
    }
}
