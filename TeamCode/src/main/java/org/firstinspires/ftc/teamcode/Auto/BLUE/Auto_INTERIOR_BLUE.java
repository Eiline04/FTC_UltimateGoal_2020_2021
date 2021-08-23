package org.firstinspires.ftc.teamcode.Auto.BLUE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.RingDetector.AdvancedCameraThread.RingPosition;

@Autonomous(group = "BLUE")
public class Auto_INTERIOR_BLUE extends LinearOpMode {
    public RingPosition ringPosition;
    OpenCvCamera webcam;
    AdvancedCameraThread cameraThread;

    LauncherWrapper launcher;
    WobbleWrapper wobbleWrapper;
    Hardware robot;
    Intake intake;

    DasPositions dasPositions;

    MecanumDrive drivetrain;
    Pose2d startPose = new Pose2d(-62.95, 23.62, Math.toRadians(180.0));

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
        sleep(2500);
        cameraThread.setState(AdvancedCameraThread.CAMERA_STATE.STREAM);

        launcher.setServoPosition(0.7f);
        wobbleWrapper.closeArm();
        wobbleWrapper.attachGrip();
        wobbleWrapper.resetWobbleRelease();

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

        toShooting = drivetrain.trajectoryBuilder(startPose, true)
                .strafeTo(new Vector2d(-40.0, 18.0), setMaxVelocity(20.0), new ProfileAccelerationConstraint(20.0))
                .splineToConstantHeading(new Vector2d(-1.0, 9.67 + 5.0), Math.toRadians(0.0), setMaxVelocity(20.0), new ProfileAccelerationConstraint(20.0)).build();

        launcher.openStopper();
        launcher.setVelocity(LauncherWrapper.shootingVelocity - 60.0, AngleUnit.DEGREES);
        drivetrain.followTrajectory(toShooting);
        sleep(300);

        launcher.launchOneRing(); //power shots go brr
        sleep(launchSleepTime);
        dasPositions.setPositionDAS(0.82);
        sleep(500);
        launcher.launchOneRing();
        sleep(300);
        dasPositions.setPositionDAS(0.78);
        sleep(500);
        launcher.launchOneRing();

        //-----------------ZERO---------------
        if (ringPosition == RingPosition.NONE) {
            buildPathsZero();
            launcher.stop();
            dasPositions.startDAS();

            drivetrain.followTrajectory(toZoneA);
            Hardware.intakeRelease.setPosition(0.667);
            wobbleWrapper.wobbleRelease();
            sleep(500);

            drivetrain.followTrajectory(toBounceBackA);
            intake.startIntake();
            sleep(100);
            drivetrain.followTrajectory(collectA);
            launcher.setVelocity(LauncherWrapper.shootingVelocity, AngleUnit.DEGREES);
            sleep(800);
            intake.stopIntake();
            drivetrain.followTrajectory(toShooting2A);

            launcher.launchOneRing();
            sleep(launchSleepTime);
            launcher.launchOneRing();
            sleep(launchSleepTime);
            launcher.launchOneRing();
            sleep(launchSleepTime);
            launcher.launchOneRing();
            sleep(500);
            launcher.stop();

            drivetrain.followTrajectory(parkA);
        }

        //------------------ONE----------------
        if (ringPosition == RingPosition.ONE) {
            buildPathsOne();

            launcher.stop();
            dasPositions.startDAS();

            drivetrain.followTrajectory(toZoneB);
            Hardware.intakeRelease.setPosition(0.667);
            wobbleWrapper.wobbleRelease();
            sleep(500);

            drivetrain.followTrajectory(toBounceBackB);
            intake.startIntake();
            sleep(100);
            drivetrain.followTrajectory(collectB);
            launcher.setVelocity(LauncherWrapper.shootingVelocity, AngleUnit.DEGREES);
            sleep(800);
            intake.stopIntake();
            drivetrain.followTrajectory(toShooting2B);

            launcher.launchOneRing();
            sleep(launchSleepTime);
            launcher.launchOneRing();
            sleep(launchSleepTime);
            launcher.launchOneRing();
            sleep(launchSleepTime);
            launcher.launchOneRing();
            sleep(500);
            launcher.stop();

            drivetrain.followTrajectory(parkB);
        }
        //-----------------FOUR-----------------
        if (ringPosition == RingPosition.FOUR) {
            buildPathsFour();
            launcher.stop();
            dasPositions.startDAS();

            drivetrain.followTrajectory(toZoneC);
            Hardware.intakeRelease.setPosition(0.667);
            wobbleWrapper.wobbleRelease();
            sleep(500);

            drivetrain.followTrajectory(toBounceBackC);
            intake.startIntake();
            sleep(100);
            drivetrain.followTrajectory(collectC);
            launcher.setVelocity(LauncherWrapper.shootingVelocity, AngleUnit.DEGREES);
            sleep(800);
            intake.stopIntake();
            drivetrain.followTrajectory(toShooting2C);

            launcher.launchOneRing();
            sleep(launchSleepTime);
            launcher.launchOneRing();
            sleep(launchSleepTime);
            launcher.launchOneRing();
            sleep(launchSleepTime);
            launcher.launchOneRing();
            sleep(500);
            launcher.stop();

            drivetrain.followTrajectory(parkC);
        }

        wobbleWrapper.attachGrip();
        wobbleWrapper.closeArm();
        sleep(300);
    }

    void buildPathsZero() {
        toZoneA = drivetrain.trajectoryBuilder(toShooting.end(), false)
                .lineToLinearHeading(new Pose2d(18.0, 47.0, Math.toRadians(200.0))).build();

        toBounceBackA = drivetrain.trajectoryBuilder(toZoneA.end(), true)
                .lineToLinearHeading(new Pose2d(55.0, 55.0, Math.toRadians(290.0))).build();

        collectA = drivetrain.trajectoryBuilder(toBounceBackA.end(), false)
                .lineToConstantHeading(new Vector2d(55.0, -2.0), setMaxVelocity(20.0), new ProfileAccelerationConstraint(20.0)).build();

        toShooting2A = drivetrain.trajectoryBuilder(collectA.end(), true)
                .lineToLinearHeading(new Pose2d(-12.0, 14.0, Math.toRadians(185.0))).build();

        parkA = drivetrain.trajectoryBuilder(toShooting2A.end(),true)
                .lineToLinearHeading(new Pose2d(10.0,10.0, Math.toRadians(180.0))).build();
    }

    Trajectory toZoneA, toBounceBackA, collectA, toShooting2A, parkA;

    void buildPathsOne() {
        toZoneB = drivetrain.trajectoryBuilder(toShooting.end(), false)
                .lineToLinearHeading(new Pose2d(41.0, 25.0, Math.toRadians(230.0))).build();

        toBounceBackB = drivetrain.trajectoryBuilder(toZoneB.end(), true)
                .lineToLinearHeading(new Pose2d(55.0, 55.0, Math.toRadians(290.0))).build();

        collectB = drivetrain.trajectoryBuilder(toBounceBackB.end(), false)
                .lineToConstantHeading(new Vector2d(55.0, -2.0), setMaxVelocity(20.0), new ProfileAccelerationConstraint(20.0)).build();

        toShooting2B = drivetrain.trajectoryBuilder(collectB.end(), true)
                .lineToLinearHeading(new Pose2d(-12.0, 14.0, Math.toRadians(185.0))).build();

        parkB = drivetrain.trajectoryBuilder(toShooting2B.end(),true)
                .lineToLinearHeading(new Pose2d(10.0,10.0, Math.toRadians(180.0))).build();
    }

    Trajectory toZoneB, toBounceBackB, collectB, toShooting2B, parkB;


    void buildPathsFour() {
        toZoneC = drivetrain.trajectoryBuilder(toShooting.end(), false)
                .lineToLinearHeading(new Pose2d(52.0, 50.0, Math.toRadians(180.0))).build();

        toBounceBackC = drivetrain.trajectoryBuilder(toZoneC.end(), true)
                .lineToLinearHeading(new Pose2d(55.0, 55.0, Math.toRadians(290.0))).build();

        collectC = drivetrain.trajectoryBuilder(toBounceBackC.end(), false)
                .lineToConstantHeading(new Vector2d(55.0, -2.0), setMaxVelocity(20.0), new ProfileAccelerationConstraint(20.0)).build();

        toShooting2C = drivetrain.trajectoryBuilder(collectC.end(), true)
                .lineToLinearHeading(new Pose2d(-12.0, 14.0, Math.toRadians(185.0))).build();

        parkC = drivetrain.trajectoryBuilder(toShooting2C.end(),true)
                .lineToLinearHeading(new Pose2d(10.0,10.0, Math.toRadians(180.0))).build();
    }

    Trajectory toZoneC, toBounceBackC, collectC, toShooting2C, parkC;

    MinVelocityConstraint setMaxVelocity(double maxVel) {
        return (new MinVelocityConstraint(Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL)
                , new MecanumVelocityConstraint(maxVel, DriveConstants.TRACK_WIDTH))));
    }

    public void initWebcam() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
    }
}
