package org.firstinspires.ftc.teamcode.Roadrunner.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Roadrunner.DriveConstants;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.Objects;

@Config
@Disabled
@Autonomous(group = "drive")
public class MaxAccelerationTuner extends LinearOpMode {
    public static double RUNTIME = 3.0;

    private ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        drive.setDrivePower(new Pose2d(1, 0, 0));

        timer = new ElapsedTime();
        timer.reset();
        double currentTime, prevTime;
        currentTime = prevTime = timer.milliseconds();

        double currentVel, prevVel;
        currentVel = prevVel = drive.getPoseVelocity().getX();

        double maxAcc = -1;
        while (!isStopRequested() && timer.seconds() < RUNTIME) {
            drive.updatePoseEstimate();

            currentTime = timer.milliseconds();
            double deltaTime = currentTime - prevTime;

            currentVel = drive.getPoseVelocity().getX();
            double deltaVelocity = currentVel - prevVel;

            double acceleration = deltaVelocity / deltaTime;

            if (acceleration > maxAcc) maxAcc = acceleration;

            telemetry.addData("Acceleration", acceleration);
            telemetry.update();

            prevTime = currentTime;
            prevVel = currentVel;
        }

        drive.setDrivePower(new Pose2d());

        telemetry.addData("Maximum Acceleration", maxAcc);
        telemetry.update();
        while (!isStopRequested() && opModeIsActive()) idle();
    }
}
