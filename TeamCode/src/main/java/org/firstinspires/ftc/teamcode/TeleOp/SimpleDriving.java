package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Miscellaneous.ControllerInput;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Wrappers.DasPositions;
import org.firstinspires.ftc.teamcode.Wrappers.Intake;
import org.firstinspires.ftc.teamcode.Wrappers.LauncherWrapper;
import org.firstinspires.ftc.teamcode.Wrappers.WobbleWrapper;

import static org.firstinspires.ftc.teamcode.Wrappers.LauncherWrapper.TeleOpPowerShotVelocity;
import static org.firstinspires.ftc.teamcode.Wrappers.LauncherWrapper.TeleOpShootingVelocity;

@TeleOp(group = "Main")
public class SimpleDriving extends LinearOpMode {
    ControllerInput controller1;
    ControllerInput controller2;

    Hardware robot;
    Intake intake;
    LauncherWrapper launcher;
    WobbleWrapper wobbleWrapper;
    DasPositions positionDAS;

    boolean intakeState = false, prevIntakeState = false;
    boolean gripperState = false; //false = detached
    boolean launcherState = false, prevLauncherState = false, tempPowerShots = false;
    boolean atPowerShots = false;

    int valueDAS = 0;
    /*0 --> start position
     * 3 --> right power shot
     * 2 --> middle power shot
     * 1 --> left power shot*/

    boolean wingState = true, prevWingState = true; //false = down/open ; true = up/closed

    double leftStickY, leftStickX, rightStickX, eps = 0.01, minPower = 0.2;

    int ringsToBeShot = 0;

    private final int launchSleepTime = 250;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drivetrain = new MecanumDrive(hardwareMap);
        initSubsystems();
        controller1 = new ControllerInput(gamepad1);
        controller2 = new ControllerInput(gamepad2);

        wobbleWrapper.attachGrip();
        gripperState = true;

        robot.enableBulkDataPolling();

        waitForStart();

        while (opModeIsActive()) {
            drivetrain.update();
            controller1.update();
            controller2.update();

            if (ringsToBeShot >= 1) {
                launcher.launchOneRing();
                sleep(launchSleepTime);
                ringsToBeShot--;

                handleDriving(drivetrain);

                if (ringsToBeShot == 0) {
                    launcher.closeStopper();
                    sleep(50);
                }
                continue;
            }
            handleSubsystems();
            handleDriving(drivetrain);

            prevLauncherState = launcherState;
        }
    }

    void initSubsystems() {
        robot = new Hardware();
        robot.init(hardwareMap);
        intake = new Intake(robot.staticIntake, robot.mobileIntake, robot.mopStanga, robot.mopDreapta);
        wobbleWrapper = new WobbleWrapper(robot.gripperServo, robot.armServo, robot.wobbleRelease);
        launcher = new LauncherWrapper(robot.launcherTop, robot.launcherBottom, robot.launchServo, robot.ringStopper);
        launcher.setPIDFCoeff(new PIDFCoefficients(55, 0, 0, 11.5));
        launcher.setServoPosition(0.7f);
        launcher.closeStopper();

        positionDAS = new DasPositions(robot.servoDAS);
        positionDAS.startDAS();

        wobbleWrapper.closeArm();
        wobbleWrapper.detachGrip();
        wobbleWrapper.resetWobbleRelease();
    }

    void handleSubsystems() {
        //----------------------INTAKE-----------------------------
        if (controller1.XOnce()) {
            intake.reverseIntake();
            intake.stopIntake();
            sleep(100);
            intake.startIntake();
        }

        if (controller1.AOnce()) {
            intakeState = !intakeState;
        }

        if (intakeState && !prevIntakeState) {
            //start the intake
            intake.startIntake();
        }
        if (!intakeState && prevIntakeState) {
            //stop the intake
            intake.stopIntake();
        }
        prevIntakeState = intakeState;

        //------------------------LAUNCHER & WINGS--------------------------
        handleLauncher();

        //--------------------------LAUNCH SERVO-----------------------------
        if (controller1.dpadUpOnce()) {
            if (launcher.isClosed) launcher.openStopper(); //wtf?
            if (!atPowerShots) {
                ringsToBeShot = 3;
            }
        }
        if (controller1.YOnce()) {
            launcher.openStopper();
            sleep(50);
            launcher.launchOneRing();
            launcher.closeStopper();
        }

        //--------------------------ARM SERVO-----------------------------
        if (controller2.dpadLeftOnce()) {
            wobbleWrapper.openArm();
        }
        if (controller2.dpadRightOnce()) {
            wobbleWrapper.closeArm();
        }

        //--------------------------GRIPPER SERVO-----------------------------
        if (controller2.dpadUpOnce()) {
            gripperState = !gripperState;
            if (gripperState) wobbleWrapper.attachGrip();
            if (!gripperState) wobbleWrapper.detachGrip();
        }

        if (controller2.YOnce()) {
            wobbleWrapper.setArmPosition(0.45f);
            sleep(500);
            wobbleWrapper.detachGrip();
            sleep(300);
            wobbleWrapper.closeArm();
        }
    }

    void handleLauncher() {
        //temporary
        if(controller1.rightBumperOnce()) {
            launcherState = true;
            prevLauncherState = true;
            launcher.setVelocity(TeleOpPowerShotVelocity, AngleUnit.DEGREES);
            Hardware.intakeRelease.setPosition(0.667);
        }

        if (controller1.BOnce() && !gamepad1.start && !gamepad2.start) {
            launcherState = !launcherState;
        }

        if (launcherState && !prevLauncherState) {
            launcher.setVelocity(TeleOpShootingVelocity, AngleUnit.DEGREES);
            Hardware.intakeRelease.setPosition(0.667);
        }

        if (!launcherState && prevLauncherState) {
            launcher.stop();
        }
    }

    //Ridiculously over-engineered. Accounts for misalignment of joysticks (if present) by using epsilon equality
    void handleDriving(MecanumDrive drivetrain) {
        leftStickY = -controller1.left_stick_y;
        if (Math.abs(leftStickY) < eps) leftStickY = 0;
        else {
            if (leftStickY < 0) leftStickY = Range.scale(leftStickY, -1.0, 0, -1.0, -minPower);
            if (leftStickY > 0) leftStickY = Range.scale(leftStickY, 0, 1, minPower, 1.0);
        }

        leftStickX = -controller1.left_stick_x;
        if (Math.abs(leftStickX) < eps) leftStickX = 0;
        else {
            if (leftStickX < 0) leftStickX = Range.scale(leftStickX, -1.0, 0, -1.0, -minPower);
            if (leftStickX > 0) leftStickX = Range.scale(leftStickX, 0, 1, minPower, 1.0);
        }

        double slowRotateRight = controller1.right_trigger;
        double slowRotateLeft = controller1.left_trigger;
        double finalRotation;
        if (slowRotateLeft < eps && slowRotateRight < eps) {
            //if we have no input from slow rotation just use normal
            rightStickX = -controller1.right_stick_x;
            if (Math.abs(rightStickX) < eps) rightStickX = 0;
            else {
                if (rightStickX < 0)
                    rightStickX = Range.scale(rightStickX, -1.0, 0, -1.0, -minPower);
                if (rightStickX > 0) rightStickX = Range.scale(rightStickX, 0, 1, minPower, 1.0);
            }
            finalRotation = rightStickX;
        } else {
            //we have input from slow rotation so apply it
            if (slowRotateLeft < eps) finalRotation = -0.2;
            else finalRotation = 0.2;
        }

        drivetrain.setWeightedDrivePower(new Pose2d(leftStickY, leftStickX, finalRotation));
    }
}
