package org.firstinspires.ftc.teamcode.Auto.BLUE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Auto.PoseStorage;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.RingDetector.AdvancedCameraThread;
import org.firstinspires.ftc.teamcode.RingDetector.CameraThread;
import org.firstinspires.ftc.teamcode.Roadrunner.DriveConstants;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Wrappers.DasPositions;
import org.firstinspires.ftc.teamcode.Wrappers.Intake;
import org.firstinspires.ftc.teamcode.Wrappers.LauncherWrapper;
import org.firstinspires.ftc.teamcode.Wrappers.WobbleWrapper;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.RingDetector.CameraThread.RingDeterminationPipeline.RingPosition;

@Autonomous(group = "BLUE")
public class Auto4_BLUE extends LinearOpMode {
    public RingPosition ringPosition;
    OpenCvCamera webcam;
    AdvancedCameraThread cameraThread;

    LauncherWrapper launcher;
    WobbleWrapper wobbleWrapper;
    Hardware robot;
    Intake intake;

    DasPositions dasPositions;

    MecanumDrive drivetrain;
    Pose2d startPose = new Pose2d(-62.8, 49.0, Math.toRadians(180.0));

    private final int launchSleepTime = 300;
    Trajectory toShooting;

    @Override
    public void runOpMode() throws InterruptedException {
        PoseStorage.currentPose = null; //clear pose transfer
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

        double rectHeight = AdvancedCameraThread.RingPipeline.rectHeight;
        double rectWidth = AdvancedCameraThread.RingPipeline.rectWidth;
        ringPosition = AdvancedCameraThread.getResult(rectHeight, rectWidth);

        telemetry.addData("Result", ringPosition);
        telemetry.update();

        cameraThread.setState(AdvancedCameraThread.CAMERA_STATE.KILL);

        drivetrain = new MecanumDrive(hardwareMap);
        drivetrain.setPoseEstimate(startPose);

        toShooting = drivetrain.trajectoryBuilder(startPose, true).lineToLinearHeading(new Pose2d(-12.0, 55.0, Math.toRadians(165.0))).build();

        launcher.openStopper();
        launcher.setVelocity(LauncherWrapper.shootingVelocity - 19.0, AngleUnit.DEGREES);
        drivetrain.followTrajectory(toShooting);
        sleep(launchSleepTime);

        launcher.launchOneRing();
        sleep(launchSleepTime);
        launcher.launchOneRing();
        sleep(launchSleepTime);
        launcher.launchOneRing();

        sleep(300);
        launcher.closeStopper();
        launcher.stop();

        if (ringPosition == RingPosition.NONE) {
            buildPathsZero();

            drivetrain.followTrajectory(toZoneA);
            wobbleWrapper.wobbleRelease();
            sleep(500);
            drivetrain.followTrajectory(park_A);
            sleep(500);
        }

        if (ringPosition == RingPosition.ONE) {
            buildPathsOne();

            drivetrain.followTrajectory(toZoneB);
            wobbleWrapper.wobbleRelease();
            sleep(500);

            intake.startIntake();
            drivetrain.followTrajectory(collectB);
            sleep(100);
            drivetrain.followTrajectory(forwardB);

            launcher.setVelocity(LauncherWrapper.shootingVelocity - 19.0, AngleUnit.DEGREES);
            sleep(100);

            drivetrain.followTrajectory(toShooting2B);
            intake.stopIntake();
            sleep(300);

            launcher.launchOneRing();
            sleep(launchSleepTime);
            launcher.launchOneRing();
            sleep(launchSleepTime);
            launcher.stop();

            drivetrain.followTrajectory(park_B);
        }

        if (ringPosition == RingPosition.FOUR) {
            buildPathsFour();

            drivetrain.followTrajectory(toZoneC);
            wobbleWrapper.wobbleRelease();
            sleep(500);
            drivetrain.followTrajectory(park_C);
            sleep(500);
        }

        drivetrain.updatePoseEstimate();
        PoseStorage.currentPose = drivetrain.getPoseEstimate(); //store pose
        wobbleWrapper.attachGrip();
        sleep(300);
    }

    void buildPathsZero() {
        toZoneA = drivetrain.trajectoryBuilder(toShooting.end(), true)
                .lineToLinearHeading(new Pose2d(12.0, 48.0, Math.toRadians(180.0))).build();

        park_A = drivetrain.trajectoryBuilder(toZoneA.end(), true)
                .splineToSplineHeading(new Pose2d(50.0, 16.0, Math.toRadians(90.0)), Math.toRadians(270.0))
                .splineToSplineHeading(new Pose2d(10.0, 0.0, Math.toRadians(0.0)), Math.toRadians(180.0))
                .build();
    }

    Trajectory toZoneA, park_A;

    void buildPathsOne() {
        toZoneB = drivetrain.trajectoryBuilder(toShooting.end(), true)
                .lineToLinearHeading(new Pose2d(30.0, 35.0, Math.toRadians(100.0))).build();

        collectB = drivetrain.trajectoryBuilder(toZoneB.end(), false)
                .lineToLinearHeading(new Pose2d(-5.0, 35.0,Math.toRadians(180.0))).build();

        forwardB = drivetrain.trajectoryBuilder(collectB.end(), false)
                .lineToConstantHeading(new Vector2d(-35.0, 35.0)).build();

        toShooting2B = drivetrain.trajectoryBuilder(forwardB.end(), true)
                .lineToLinearHeading(new Pose2d(-12.0, 35.0, Math.toRadians(180.0))).build();

        park_B = drivetrain.trajectoryBuilder(toZoneB.end(), true)
                .lineToLinearHeading(new Pose2d(10.0, 0.0, Math.toRadians(180.0))).build();
    }

    Trajectory toZoneB, collectB, forwardB, toShooting2B, park_B;

    void buildPathsFour() {
        toZoneC = drivetrain.trajectoryBuilder(toShooting.end(), true)
                .splineToSplineHeading(new Pose2d(60.0, 45.0, Math.toRadians(180.0)), Math.toRadians(0.0)).build();

        park_C = drivetrain.trajectoryBuilder(toZoneC.end(), true)
                .lineToConstantHeading(new Vector2d(55.0, 20.0))
                .splineToConstantHeading(new Vector2d(10.0, 10.0), Math.toRadians(180.0)).build();
    }

    Trajectory toZoneC, park_C;

    MinVelocityConstraint setMaxVelocity(double maxVel) {
        return (new MinVelocityConstraint(Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL)
                , new MecanumVelocityConstraint(maxVel, DriveConstants.TRACK_WIDTH))));
    }

    public void initWebcam() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
    }
}
