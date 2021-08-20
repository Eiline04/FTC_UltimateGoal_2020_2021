package org.firstinspires.ftc.teamcode.Miscellaneous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;

@Config
@TeleOp
@Disabled
public class RotatePIDTuner extends LinearOpMode {

    Hardware robot;
    ControllerInput controller;
    MecanumDrive drivetrain;

    public static double kP = 5;
    public static double kI = 0;
    public static double kD = 0;

    public static double rotateTo_DEG = 180.0;
    public double maxAllowedError = 1; //radians
    public double maxPower = 0.3;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Hardware();
        robot.init(hardwareMap);
        controller = new ControllerInput(gamepad1);
        drivetrain = new MecanumDrive(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();

        while (!isStopRequested()) {
            controller.update();
            drivetrain.update();

            if (controller.rightBumperOnce()) {
                telemetry.log().clear();
                PIDCoefficients pidCoefficients = new PIDCoefficients(kP, kI, kD);
                PIDFController newPID = new PIDFController(pidCoefficients);
                newPID.setInputBounds(0.0, 2.0 * Math.PI);
                //newPID.setOutputBounds(0.0, 1.0); //is this correct?
                newPID.setTargetPosition(Math.toRadians(rotateTo_DEG));

                //rotate the specified amount

                while (Math.abs(newPID.getLastError()) > maxAllowedError && !controller.AOnce()) {
                    controller.update();
                    drivetrain.update();
                    double heading = drivetrain.getPoseEstimate().getHeading();
                    double correction = newPID.update(heading);

                    telemetry.addData("Correction", correction);
                    telemetry.addData("Current Heading", Math.toDegrees(heading));
                    telemetry.addData("Target Heading", rotateTo_DEG);
                    telemetry.update();

//                    robot.frontLeftWheel.setPower(-maxPower);
//                    robot.frontRightWheel.setPower(maxPower);
//                    robot.backLeftWheel.setPower(-maxPower);
//                    robot.backRightWheel.setPower(maxPower);
                }
                robot.frontLeftWheel.setPower(0);
                robot.frontRightWheel.setPower(0);
                robot.backLeftWheel.setPower(0);
                robot.backRightWheel.setPower(0);

                telemetry.addLine("Rotation Complete");
                telemetry.update();

            } else {
                double heading = drivetrain.getPoseEstimate().getHeading();

                telemetry.addData("Current Heading", Math.toDegrees(heading));
                telemetry.addData("Target Heading", rotateTo_DEG);
                telemetry.update();
            }
        }
    }
}
