package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Miscellaneous.ControllerInput;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Wrappers.DasPositions;
import org.firstinspires.ftc.teamcode.Wrappers.Intake;
import org.firstinspires.ftc.teamcode.Wrappers.LauncherWrapper;
import org.firstinspires.ftc.teamcode.Wrappers.WobbleWrapper;

@TeleOp(group = "Misc")
public class TestTeleOp extends LinearOpMode {

    LauncherWrapper launcher;
    WobbleWrapper wobbleWrapper;
    DasPositions dasPositions;
    Hardware robot;
    Intake intake;

    ControllerInput controller1,controller2;

    MecanumDrive drivetrain;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Hardware();
        robot.init(hardwareMap);
        wobbleWrapper = new WobbleWrapper(robot.gripperServo, robot.armServo, robot.wobbleRelease);
        wobbleWrapper.closeArm();
        launcher = new LauncherWrapper(robot.launcherTop, robot.launcherBottom, robot.launchServo, robot.ringStopper);
        launcher.setPIDFCoeff(new PIDFCoefficients(55, 0, 0, 11.5));

        controller1 = new ControllerInput(gamepad1);
        controller2 = new ControllerInput(gamepad2);

        dasPositions = new DasPositions(robot.servoDAS);
        dasPositions.setPositionDAS(0.7);

        waitForStart();

        launcher.openStopper();
        while (opModeIsActive()) {
            controller1.update();

            if(controller1.AOnce()) {
                launcher.setVelocity(LauncherWrapper.shootingVelocity, AngleUnit.DEGREES);
            }

            if(controller1.BOnce()) {
                launcher.stop();
            }

            if(controller1.YOnce()) {
                launcher.launchOneRing();
                sleep(300);
            }
        }
    }
}
