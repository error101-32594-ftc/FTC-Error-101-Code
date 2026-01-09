// Author: ElectraBytes04
// Last Modified: 2025-11-16

package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

public class DiagnosticLogger implements Runnable
{
    private final Telemetry telemetry;
    private final VoltageSensor voltageSensor;
    private final DcMotor[] dcMotors;
    private final DcMotorEx[] dcMotorExs;
    private final CRServo[] crServos;
    private final IMU imu;

    private final BufferedCsvWriter bufferedCsvWriter;
    private volatile Boolean paused = true;
    private volatile Boolean stopped = false;

    private final long msSinceEpoch = System.currentTimeMillis();

    public DiagnosticLogger(
            Telemetry providedTelemetry, HardwareMap providedHardwareMap,
            String[] providedDcMotorNames, String[] providedDcMotorExNames,
            String[] providedCRServoNames, String providedImuName,
            IMU.Parameters providedImuParams
            ) throws IOException {

        String filename = String.format("/sdcard/FIRST/DiagnosticLogs/%s.csv",
                getCurrentDate());
        StringBuilder header = new StringBuilder();
        header.append("timestamp,voltage,");

        this.telemetry = providedTelemetry;

        this.voltageSensor = providedHardwareMap.voltageSensor
                .iterator().next();
        
        // Building the header string:
        if(providedDcMotorNames != null)
        {
            this.dcMotors = new DcMotor[providedDcMotorNames.length];
            for(int i = 0; i < providedDcMotorNames.length; i++) {
                this.dcMotors[i] = providedHardwareMap.get(DcMotor.class,
                        providedDcMotorNames[i]);
            }
            for(String dcMotorName : providedDcMotorNames)
            {
                header.append(dcMotorName).append("_port,");
                header.append(dcMotorName).append("_power,");
                header.append(dcMotorName).append("_position,");
            }
        } else { this.dcMotors = null; }

        if(providedDcMotorExNames != null)
        {
            this.dcMotorExs = new DcMotorEx[providedDcMotorExNames.length];
            for(int i = 0; i < providedDcMotorExNames.length; i++) {
                this.dcMotorExs[i] = providedHardwareMap.get(DcMotorEx.class,
                        providedDcMotorExNames[i]);
            }
            for(String dcMotorExName : providedDcMotorExNames)
            {
                header.append(dcMotorExName).append("_port,");
                header.append(dcMotorExName).append("_power,");
                header.append(dcMotorExName).append("_currentposition,");
                header.append(dcMotorExName).append("_targetposition,");
                header.append(dcMotorExName).append("_velocity,");
                header.append(dcMotorExName).append("_current,");
            }
        } else { this.dcMotorExs = null; }

        if(providedCRServoNames != null)
        {
            this.crServos = new CRServo[providedCRServoNames.length];
            for(int i = 0; i < providedCRServoNames.length; i++) {
                this.crServos[i] = providedHardwareMap.get(CRServo.class,
                        providedCRServoNames[i]);
            }
            for(String crServoName : providedCRServoNames)
            {
                header.append(crServoName).append("_port,");
                header.append(crServoName).append("_power,");
                header.append(crServoName).append("_direction,");
            }
        } else { this.crServos = null; }

        if(providedImuName != null)
        {
            this.imu = providedHardwareMap.get(IMU.class, providedImuName);
            header.append("imu_yaw,imu_pitch,imu_roll");
            imu.initialize(providedImuParams);
        } else { this.imu = null; }
        

        String csvHeader = header.toString().replaceAll(",+$", "");
        this.bufferedCsvWriter = new BufferedCsvWriter(filename, csvHeader);
    }

    public String getCurrentDate()
    {
        return new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss", Locale.ENGLISH)
                .format(new Date());
    }

    private static class BufferedCsvWriter
    {
        final FileWriter fileWriter;
        private final BufferedWriter bufferedWriter;

        public BufferedCsvWriter(
                String filename, String header
                ) throws IOException
        {
            File tmp = new File(filename);
            if(!tmp.exists())
            {
                tmp.getParentFile().mkdirs();
            }
            fileWriter = new FileWriter(filename, true);
            bufferedWriter = new BufferedWriter(fileWriter);

            bufferedWriter.write(header);
            bufferedWriter.newLine();
            bufferedWriter.flush();
        }

        public void write(String line) throws IOException
        {
            bufferedWriter.write(line);
            bufferedWriter.newLine();
            bufferedWriter.flush();
        }

        public void close() throws IOException
        {
            bufferedWriter.close();
        }
    }

    public String buildCsvLine()
    {
        StringBuilder row = new StringBuilder();
        DecimalFormat df = new DecimalFormat("#.####");
        row.append(System.currentTimeMillis() - msSinceEpoch).append(",");
        row.append(df.format(voltageSensor.getVoltage())).append(",");

        if(dcMotors != null)
        {
            for(DcMotor dcMotor : dcMotors)
            {
                row.append(dcMotor.getPortNumber()).append(",");
                row.append(df.format(dcMotor.getPower())).append(",");
                row.append(df.format(dcMotor.getCurrentPosition())).append(",");
            }
        }

        if(dcMotorExs != null)
        {
            for(DcMotorEx dcMotorEx : dcMotorExs)
            {
                row.append(dcMotorEx.getPortNumber()).append(",");
                row.append(
                        df.format(dcMotorEx.getPower())
                ).append(",");
                row.append(
                        df.format(dcMotorEx.getCurrentPosition())
                ).append(",");
                row.append(
                        df.format(dcMotorEx.getTargetPosition())
                ).append(",");
                row.append(
                        df.format(dcMotorEx.getVelocity())
                ).append(",");
                row.append(
                        df.format(dcMotorEx.getCurrent(CurrentUnit.MILLIAMPS))
                ).append(",");
            }
        }

        if(crServos != null)
        {
            for(CRServo crServo : crServos)
            {
                row.append(df.format(crServo.getPortNumber())).append(",");
                row.append(df.format(crServo.getPower())).append(",");
                row.append(df.format(crServo.getDirection())).append(",");
            }
        }

        if(imu != null)
        {
            YawPitchRollAngles robotOrientation =
                    imu.getRobotYawPitchRollAngles();
            row.append(df.format(robotOrientation.getYaw())).append(",");
            row.append(df.format(robotOrientation.getPitch())).append(",");
            row.append(df.format(robotOrientation.getRoll())).append(",");
        }
        return row.toString();
    }

    public void run()
    {
        while(!stopped)
        {
            if(!paused)
            {
                try
                {
                    bufferedCsvWriter.write(buildCsvLine());
                    Thread.sleep(100);
                } catch(IOException e)
                {
                    telemetry.addData("Logger Write Error", e.getMessage());
                    telemetry.update();
                } catch(InterruptedException e)
                {
                    Thread.currentThread().interrupt();
                }
            }
        }
        try
        {
            bufferedCsvWriter.close();
        } catch(IOException e)
        {
            telemetry.addData("Logger Close Error", e.getMessage());
            telemetry.update();
        }
    }

    public void pauseRun()
    {
        this.paused = true;
    }
    public void resumeRun()
    {
        this.paused = false;
    }
    public void stopRun()
    {
        this.stopped = true;
    }
}
