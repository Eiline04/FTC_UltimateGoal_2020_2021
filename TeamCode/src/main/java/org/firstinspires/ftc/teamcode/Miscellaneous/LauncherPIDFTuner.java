package org.firstinspires.ftc.teamcode.Miscellaneous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Intake;
import org.firstinspires.ftc.teamcode.Wrappers.LauncherWrapper;

import static org.firstinspires.ftc.teamcode.Wrappers.LauncherWrapper.TeleOpShootingVelocity;

@Config
@Autonomous(group = "drive")
public class LauncherPIDFTuner extends LinearOpMode {

    Hardware robot;
    LauncherWrapper launcherWrapper;
    ControllerInput controller;
    Intake intake;

    public static double kP = 25;
    public static double kI = 0;
    public static double kD = 0;
    public static double f = 11.5;

    public static double targetVelocity = LauncherWrapper.TeleOpShootingVelocity;

    public static int sleepTime = 300;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Hardware();
        robot.init(hardwareMap);
        launcherWrapper = new LauncherWrapper(robot.launcherTop, robot.launcherBottom, robot.launchServo, robot.ringStopper);
        intake = new Intake(robot.staticIntake, robot.mobileIntake, robot.mopStanga, robot.mopDreapta);
        controller = new ControllerInput(gamepad1);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //Default values: kP: 10  kI: 3  kD: 0 F: 0

//        PIDFCoefficients defaultPID = launcherWrapper.launcherBottom.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
//        telemetry.addLine("Current Values:");
//        telemetry.addData("kP:", defaultPID.p);
//        telemetry.addData("kI:", defaultPID.i);
//        telemetry.addData("kD:", defaultPID.d);
//        telemetry.addData("F:", defaultPID.f);
//        telemetry.addLine();
//        telemetry.addLine("Press Play when ready");
//        telemetry.update();

        waitForStart();

        while (!isStopRequested()) {
            controller.update();
            if (controller.rightBumperOnce()) {
                PIDFCoefficients newPID = new PIDFCoefficients(kP, kI, kD, f);
                launcherWrapper.setPIDFCoeff(newPID);
                sleep(100);
                launcherWrapper.setVelocity(targetVelocity, AngleUnit.DEGREES);
            }
            if (controller.leftBumperOnce()) {
                launcherWrapper.setVelocity(0, AngleUnit.DEGREES);
            }

            if(controller.YOnce()) {
                launchOneRing();
                telemetry.addData("Current Velocity", launcherWrapper.getAngularVelocity());
                telemetry.addData("Target Velocity",targetVelocity);
                telemetry.update();
                sleep(sleepTime);
                telemetry.addData("Current Velocity", launcherWrapper.getAngularVelocity());
                telemetry.addData("Target Velocity",targetVelocity);
                telemetry.update();
                launchOneRing();
                telemetry.addData("Current Velocity", launcherWrapper.getAngularVelocity());
                telemetry.addData("Target Velocity",targetVelocity);
                telemetry.update();
                sleep(sleepTime);
                telemetry.addData("Current Velocity", launcherWrapper.getAngularVelocity());
                telemetry.addData("Target Velocity",targetVelocity);
                telemetry.update();
                launchOneRing();
                telemetry.addData("Current Velocity", launcherWrapper.getAngularVelocity());
                telemetry.addData("Target Velocity",targetVelocity);
                telemetry.update();
                sleep(sleepTime);
                telemetry.addData("Current Velocity", launcherWrapper.getAngularVelocity());
                telemetry.addData("Target Velocity",targetVelocity);
                telemetry.update();
            }

            if(controller.BOnce()) {
                launchOneRing();
            }

            if(controller.dpadUpOnce()) intake.startIntake();
            if(controller.dpadDownOnce()) intake.stopIntake();

            telemetry.addData("Current Velocity", launcherWrapper.getAngularVelocity());
            telemetry.addData("Target Velocity",targetVelocity);
            telemetry.update();

        }
    }
    void launchOneRing() {
        launcherWrapper.setServoPosition(0.5f);
        sleep(150);
        launcherWrapper.setServoPosition(0.1f);
    }
}
