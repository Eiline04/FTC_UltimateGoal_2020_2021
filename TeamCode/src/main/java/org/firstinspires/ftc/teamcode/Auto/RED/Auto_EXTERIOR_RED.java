package org.firstinspires.ftc.teamcode.Auto.RED;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.RingDetector.AdvancedCameraThread;
import org.firstinspires.ftc.teamcode.Roadrunner.DriveConstants;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Wrappers.DasPositions;
import org.firstinspires.ftc.teamcode.Wrappers.Intake;
import org.firstinspires.ftc.teamcode.Wrappers.LauncherWrapper;
import org.firstinspires.ftc.teamcode.Wrappers.WobbleWrapper;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import static org.firstinspires.ftc.teamcode.RingDetector.AdvancedCameraThread.RingPosition;

import java.util.Arrays;

@Autonomous(group = "RED")
public class Auto_EXTERIOR_RED extends LinearOpMode {
    public static volatile RingPosition ringPosition;
    OpenCvCamera webcam;
    AdvancedCameraThread cameraThread;

    LauncherWrapper launcher;
    WobbleWrapper wobbleWrapper;
    Hardware robot;
    Intake intake;

    DasPositions dasPositions;

    MecanumDrive drivetrain;
    Pose2d startPose = new Pose2d(-62.8, -49.0, Math.toRadians(180.0));

    private final int launchSleepTime = 400;
    Trajectory toShooting;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Hardware();
        robot.init(hardwareMap);
        launcher = new LauncherWrapper(robot.launcherTop, robot.launcherBottom, robot.launchServo, robot.ringStopper);
        launcher.closeStopper();
        launcher.setPIDFCoeff(new PIDFCoefficients(55, 0, 0, 11.5));
        wobbleWrapper = new WobbleWrapper(robot.gripperServo, robot.armServo, robot.wobbleRelease);
        intake = new Intake(robot.staticIntake, robot.mobileIntake, robot.mopStanga, robot.mopDreapta);

        initWebcam();
        sleep(1000);
        cameraThread = new AdvancedCameraThread(webcam);
        Thread cameraRunner = new Thread(cameraThread);
        cameraRunner.start();

        cameraThread.setState(AdvancedCameraThread.CAMERA_STATE.INIT);
        sleep(1000);
        cameraThread.setState(AdvancedCameraThread.CAMERA_STATE.STREAM);

        launcher.setServoPosition(0.7f);
        wobbleWrapper.closeArm();
        wobbleWrapper.attachGrip();

        robot.enableBulkDataPolling();

        dasPositions = new DasPositions(robot.servoDAS);
        dasPositions.startDAS();

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        double rectHeight = AdvancedCameraThread.rectHeight;
        double rectWidth = AdvancedCameraThread.rectWidth;
        ringPosition = AdvancedCameraThread.getResult(rectHeight, rectWidth);

        telemetry.addData("Result", ringPosition);
        telemetry.update();

        cameraThread.setState(AdvancedCameraThread.CAMERA_STATE.KILL);

        drivetrain = new MecanumDrive(hardwareMap);
        drivetrain.setPoseEstimate(startPose);

        toShooting = drivetrain.trajectoryBuilder(startPose, true).lineToLinearHeading(new Pose2d(-12.0, -55.0, Math.toRadians(186.0)),
                setMaxVelocity(25.0), new ProfileAccelerationConstraint(25.0)).build();

        launcher.openStopper();
        launcher.setVelocity(LauncherWrapper.shootingVelocity, AngleUnit.DEGREES);
        drivetrain.followTrajectory(toShooting);
        sleep(300);

        launcher.launchOneRing();
        sleep(launchSleepTime);
        launcher.launchOneRing();
        sleep(launchSleepTime);
        launcher.launchOneRing();

        sleep(200);
        launcher.closeStopper();
        launcher.stop();

        //-----------------ZERO---------------
        if (ringPosition == RingPosition.NONE) {
            buildPathsZero();

            drivetrain.followTrajectory(toZoneA);
            Hardware.intakeRelease.setPosition(0.667);
            sleep(100);
            wobbleWrapper.detachGrip();
            sleep(300);
            wobbleWrapper.closeArm();
            sleep(400);
            drivetrain.followTrajectory(toWait);

            sleep(10000); //wait cause we have nothing better to do

            drivetrain.followTrajectory(park_A);
        }

        //-----------------ONE---------------
        if (ringPosition == RingPosition.ONE) {
            buildPathsOne();

            drivetrain.followTrajectory(toZoneB);
            Hardware.intakeRelease.setPosition(0.667);
            sleep(100);
            wobbleWrapper.detachGrip();
            sleep(300);
            wobbleWrapper.closeArm();

            drivetrain.followTrajectory(collectB);
            intake.startIntake();
            sleep(100);
            drivetrain.followTrajectory(forwardB);
            sleep(500);

            launcher.setVelocity(LauncherWrapper.shootingVelocity, AngleUnit.DEGREES);
            launcher.openStopper();
            sleep(500);

            drivetrain.followTrajectory(toShooting2B);
            intake.stopIntake();

            launcher.launchOneRing();
            sleep(launchSleepTime);
            launcher.launchOneRing();
            sleep(300);
            launcher.stop();

            drivetrain.followTrajectory(parkB);
        }

        //-----------------FOUR---------------
        if (ringPosition == RingPosition.FOUR) {
            buildPathsFour();

            dasPositions.startDAS();
            drivetrain.followTrajectory(toZoneC);
            Hardware.intakeRelease.setPosition(0.667);
            sleep(100);
            wobbleWrapper.detachGrip();
            sleep(300);
            wobbleWrapper.closeArm();

            drivetrain.followTrajectory(collectC);
            intake.startIntake();
            sleep(100);

            launcher.setVelocity(LauncherWrapper.shootingVelocity, AngleUnit.DEGREES);
            launcher.openStopper();

            //go forward, stop, go back and shoot, go forward to collect the rest
            drivetrain.followTrajectory(forwardC);
            //launcher.setVelocity(LauncherWrapper.shootingVelocity, AngleUnit.DEGREES);
            sleep(100);
            drivetrain.followTrajectory(backwardC);
            intake.stopIntake();
            sleep(700);

            //launch two rings
            launcher.launchOneRing();
            sleep(launchSleepTime);
            launcher.launchOneRing();
            intake.startIntake();
            sleep(100);

            drivetrain.followTrajectory(forwardAgainC);
            drivetrain.followTrajectory(toShooting2C);
            intake.stopIntake();
            sleep(200);

            launcher.launchOneRing();
            sleep(launchSleepTime);
            launcher.launchOneRing();
            sleep(launchSleepTime);
            launcher.launchOneRing();
            sleep(launchSleepTime);
            launcher.launchOneRing();

            drivetrain.followTrajectory(parkC);
            launcher.stop();
        }

        wobbleWrapper.closeArm();
        wobbleWrapper.attachGrip();
    }

    void buildPathsZero() {
        toZoneA = drivetrain.trajectoryBuilder(toShooting.end(), false)
                .addTemporalMarker(0.50, 0.0, () -> {
                    wobbleWrapper.openArm();
                })
                .lineToLinearHeading(new Pose2d(4.0, -53.0, Math.toRadians(195.0)),
                        setMaxVelocity(30.0), new ProfileAccelerationConstraint(20.0)).build();

        toWait = drivetrain.trajectoryBuilder(toZoneA.end(), false)
                .lineToLinearHeading(new Pose2d(-30.0, -60.0, Math.toRadians(180.0))).build();

        park_A = drivetrain.trajectoryBuilder(toWait.end(), true)
                .lineToConstantHeading(new Vector2d(10.0, -39.0)).build();
    }

    Trajectory toZoneA, toWait, park_A;

    void buildPathsOne() {
        toZoneB = drivetrain.trajectoryBuilder(toShooting.end(), true)
                .addTemporalMarker(0.50, 0.0, () -> {
                    wobbleWrapper.openArm();
                })
                .lineToLinearHeading(new Pose2d(25.0, -36.0, Math.toRadians(250.0)), setMaxVelocity(25.0), new ProfileAccelerationConstraint(25.0)).build();

        collectB = drivetrain.trajectoryBuilder(toZoneB.end(), false)
                .lineToLinearHeading(new Pose2d(-5.0, -35.0, Math.toRadians(180.0)), setMaxVelocity(25.0), new ProfileAccelerationConstraint(25.0)).build();

        forwardB = drivetrain.trajectoryBuilder(collectB.end(), false)
                .lineToConstantHeading(new Vector2d(-35.0, -35.0), setMaxVelocity(25.0), new ProfileAccelerationConstraint(20.0)).build();

        toShooting2B = drivetrain.trajectoryBuilder(forwardB.end(), true)
                .lineToLinearHeading(new Pose2d(-12.0, -35.0, Math.toRadians(172.0)), setMaxVelocity(25.0), new ProfileAccelerationConstraint(25.0)).build();

        parkB = drivetrain.trajectoryBuilder(toShooting2B.end(), true)
                .lineToLinearHeading(new Pose2d(10.0, -55.0, Math.toRadians(180.0)), setMaxVelocity(35.0), new ProfileAccelerationConstraint(30.0)).build();
    }

    Trajectory toZoneB, collectB, forwardB, toShooting2B, parkB;

    void buildPathsFour() {
        toZoneC = drivetrain.trajectoryBuilder(toShooting.end(), true)
                .addTemporalMarker(0.50, 0.0, () -> {
                    wobbleWrapper.openArm();
                })
                .lineToLinearHeading(new Pose2d(50.0, -55.0, Math.toRadians(215.0))).build();

        collectC = drivetrain.trajectoryBuilder(toZoneC.end(), false)
                .lineToLinearHeading(new Pose2d(-5.0, -35.0, Math.toRadians(180.0))).build();

        forwardC = drivetrain.trajectoryBuilder(collectC.end(), false)
                .forward(12.0, setMaxVelocity(25.0), new ProfileAccelerationConstraint(25.0)).build();

        backwardC = drivetrain.trajectoryBuilder(forwardC.end(), true)
                .lineToLinearHeading(new Pose2d(-12.0, -35.0, Math.toRadians(171.0)), setMaxVelocity(15.0), new ProfileAccelerationConstraint(15.0)).build();
                //.back(5.0, setMaxVelocity(15.0), new ProfileAccelerationConstraint(15.0)).build();

        forwardAgainC = drivetrain.trajectoryBuilder(backwardC.end(), false)
                .lineToLinearHeading(new Pose2d(-42.0, -35.0, Math.toRadians(180.0)), setMaxVelocity(15.0), new ProfileAccelerationConstraint(15.0)).build();
                //.forward(30.0, setMaxVelocity(15.0), new ProfileAccelerationConstraint(15.0)).build();

        toShooting2C = drivetrain.trajectoryBuilder(forwardAgainC.end(), true)
                .lineToLinearHeading(new Pose2d(-12.0, -35.0, Math.toRadians(172.0))).build();

        parkC = drivetrain.trajectoryBuilder(toShooting2C.end(), true)
                .lineToLinearHeading(new Pose2d(10.0, -55.0, Math.toRadians(180.0))).build();
    }

    Trajectory toZoneC, collectC, forwardC, backwardC, forwardAgainC, toShooting2C, parkC;

    MinVelocityConstraint setMaxVelocity(double maxVel) {
        return (new MinVelocityConstraint(Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL)
                , new MecanumVelocityConstraint(maxVel, DriveConstants.TRACK_WIDTH))));
    }

    public void initWebcam() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
    }
}
