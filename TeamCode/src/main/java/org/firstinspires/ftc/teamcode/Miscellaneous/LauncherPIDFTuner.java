package org.firstinspires.ftc.teamcode.Miscellaneous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Intake;
import org.firstinspires.ftc.teamcode.Wrappers.LauncherWrapper;

/**
 * OpMode for tuning the flywheel PIDF values. Uses a separate thread to write
 * continuously to telemetry. Best used with FTC Dashboard since you can plot the angular
 * velocity in time and see the response.
 */

@Config
@TeleOp
@Disabled
public class LauncherPIDFTuner extends LinearOpMode {

    Hardware robot;
    LauncherWrapper launcherWrapper;
    ControllerInput controller;
    Intake intake;

    public static double kP = 55;
    public static double kI = 0;
    public static double kD = 0;
    public static double f = 11.5;

    public static double targetVelocity = 620;

    public static int sleepTime = 300;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Hardware();
        robot.init(hardwareMap);
        launcherWrapper = new LauncherWrapper(robot.launcherTop, robot.launcherBottom, robot.launchServo, robot.ringStopper);
        intake = new Intake(robot.staticIntake, robot.mobileIntake, robot.leftOmni, robot.rightOmni);
        controller = new ControllerInput(gamepad1);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //Default values: kP: 10  kI: 3  kD: 0 F: 0

//        PIDFCoefficients defaultPID = launcherWrapper.launcherBottom.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
//        telemetry.addLine("Current Values:");
//        telemetry.addData("kP:", defaultPID.p);
//        telemetry.addData("kI:", defaultPID.i);
//        telemetry.addData("kD:", defaultPID.d);
//        telemetry.addData("F:", defaultPID.f);

        telemetry.addLine();
        telemetry.addLine("Press Play when ready");
        telemetry.update();
        waitForStart();

        telemetryThread threadObj = new telemetryThread();
        Thread thread1 = new Thread(threadObj);
        thread1.start();

        while (opModeIsActive()) {
            controller.update();

            if (controller.rightBumperOnce()) {
                PIDFCoefficients newPID = new PIDFCoefficients(kP, kI, kD, f);
                launcherWrapper.setPIDFCoeff(newPID);
                sleep(100);
                launcherWrapper.setVelocity(targetVelocity, AngleUnit.DEGREES);
            }
            if (controller.leftBumperOnce()) {
                launcherWrapper.stop();
            }

            if (controller.YOnce()) {
                launcherWrapper.launchOneRing();
                sleep(sleepTime);
                launcherWrapper.launchOneRing();
                sleep(sleepTime);
                launcherWrapper.launchOneRing();
                sleep(sleepTime);
            }

            if (controller.BOnce()) {
                launcherWrapper.launchOneRing();
            }

            if (controller.dpadUpOnce()) intake.startIntake();
            if (controller.dpadDownOnce()) intake.stopIntake();
        }
    }

    class telemetryThread implements Runnable {
        @Override
        public void run() {
            telemetry.log().clear();
            while (opModeIsActive()) {
                telemetry.addData("Current Velocity", launcherWrapper.getAngularVelocity());
                telemetry.addData("Target Velocity", targetVelocity);
                telemetry.update();
                sleep(3);
            }

        }
    }
}
