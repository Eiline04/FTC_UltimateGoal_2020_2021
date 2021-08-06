package org.firstinspires.ftc.teamcode.Auto;

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

@Autonomous
public class Auto3_RED extends LinearOpMode {
    public static volatile CameraThread.RingDeterminationPipeline.RingPosition ringPosition;
    OpenCvCamera webcam;
    CameraThread cameraThread;

    LauncherWrapper launcher;
    WobbleWrapper wobbleWrapper;
    Hardware robot;
    Intake intake;

    DasPositions dasPositions;

    MecanumDrive drivetrain;
    Pose2d startPose = new Pose2d(-62.95, -23.62, Math.toRadians(180.0));

    private final int launchSleepTime = 300;
    Trajectory toShooting;

    @Override
    public void runOpMode() throws InterruptedException {
        PoseStorage.currentPose = null; //clear pose transfer
        robot = new Hardware();
        robot.init(hardwareMap);
        launcher = new LauncherWrapper(robot.launcherTop, robot.launcherBottom, robot.launchServo, robot.ringStopper);
        launcher.closeStopper();
        launcher.setPIDFCoeff(new PIDFCoefficients(25, 0, 0, 11.5));
        wobbleWrapper = new WobbleWrapper(robot.gripperServo, robot.armServo);
        intake = new Intake(robot.staticIntake, robot.mobileIntake, robot.mopStanga, robot.mopDreapta);

        launcher.setPIDFCoeff(new PIDFCoefficients(25, 0, 0, 11.5));

        initWebcam();
        sleep(1000);
        cameraThread = new CameraThread(webcam);
        Thread cameraRunner = new Thread(cameraThread);
        cameraRunner.start();

        cameraThread.setState(CameraThread.CAMERA_STATE.INIT);
        sleep(2500);
        cameraThread.setState(CameraThread.CAMERA_STATE.STREAM);
        sleep(1000);

        launcher.setServoPosition(0.7f);
        wobbleWrapper.closeArm();
        wobbleWrapper.attachGrip();

        //robot.enableBulkDataPolling();

        dasPositions = new DasPositions(robot.servoDAS);
        //dasPositions.startDAS();
        dasPositions.setPositionDAS(0.705 - 0.01); //0.705

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();
        launcher.openStopper();

        if (isStopRequested()) return;

        cameraThread.setState(CameraThread.CAMERA_STATE.DETECT);
        sleep(50);
        cameraThread.setState(CameraThread.CAMERA_STATE.KILL);

        telemetry.addData("Result", ringPosition);
        telemetry.update();

        drivetrain = new MecanumDrive(hardwareMap);
        drivetrain.setPoseEstimate(startPose);

        toShooting = drivetrain.trajectoryBuilder(startPose, true).lineToLinearHeading(new Pose2d(-12.0, -14.0, Math.toRadians(180.0))).build();

        launcher.setVelocity(625, AngleUnit.DEGREES); //540
        drivetrain.followTrajectory(toShooting);
        sleep(launchSleepTime);

        launcher.launchOneRing();
        sleep(launchSleepTime);
        launcher.launchOneRing();
        sleep(launchSleepTime);
        launcher.launchOneRing();

        sleep(300);
        launcher.closeStopper();
        launcher.setPower(0);

        //-----------------ZERO---------------
        if (ringPosition == CameraThread.RingDeterminationPipeline.RingPosition.NONE) {
            buildPathsZero();

            drivetrain.followTrajectory(toZoneA);
            sleep(100);
            wobbleWrapper.detachGrip();
            sleep(300);
            wobbleWrapper.closeArm();
            drivetrain.followTrajectory(park_A);
            sleep(500);

            dasPositions.startDAS();
        }

        //------------------ONE----------------
        if (ringPosition == CameraThread.RingDeterminationPipeline.RingPosition.ONE) {
            buildPathsOne();

            drivetrain.followTrajectory(toZoneB);
            sleep(100);
            wobbleWrapper.detachGrip();
            sleep(300);
            wobbleWrapper.closeArm();

            intake.startIntake();
            drivetrain.followTrajectory(collectB);

            launcher.setVelocity(620, AngleUnit.DEGREES);
            launcher.openStopper();
            sleep(200);

            launcher.setVelocity(630, AngleUnit.DEGREES);
            sleep(100);

            drivetrain.followTrajectory(toShooting2B);
            intake.stopIntake();
            sleep(300);

            launcher.launchOneRing();
            sleep(launchSleepTime);
            launcher.launchOneRing();
            sleep(300);
            launcher.stop();

            drivetrain.followTrajectory(parkB);
        }
        //-----------------FOUR-----------------
        if (ringPosition == CameraThread.RingDeterminationPipeline.RingPosition.FOUR) {
            buildPathsFour();

            drivetrain.followTrajectory(toZoneC);
            sleep(100);
            wobbleWrapper.detachGrip();
            sleep(300);
            wobbleWrapper.closeArm();

            drivetrain.followTrajectory(collectC);
            intake.startIntake();
            sleep(100);

            //drivetrain.followTrajectory(forwardC);

            launcher.setVelocity(622, AngleUnit.DEGREES);
            launcher.openStopper();
            sleep(100);


            drivetrain.followTrajectory(toShooting2C);
            intake.stopIntake();
            sleep(200);

            launcher.launchOneRing();
            sleep(launchSleepTime);
            launcher.launchOneRing();
            sleep(300);
            launcher.stop();

            drivetrain.followTrajectory(parkC);

            dasPositions.startDAS();
        }

        wobbleWrapper.attachGrip();
        drivetrain.updatePoseEstimate();
        PoseStorage.currentPose = drivetrain.getPoseEstimate(); //store pose
        sleep(300);
    }

    void buildPathsZero() {
        toZoneA = drivetrain.trajectoryBuilder(toShooting.end(), false)
                .addTemporalMarker(0.50, 0.0, () -> {
                    wobbleWrapper.openArm();
                })
                .lineToLinearHeading(new Pose2d(12.0, -50.0, Math.toRadians(180.0))).build();

        park_A = drivetrain.trajectoryBuilder(toZoneA.end(), true)
                .strafeRight(60.0).build();
    }

    Trajectory toZoneA, park_A;

    void buildPathsOne() {
        toZoneB = drivetrain.trajectoryBuilder(toShooting.end(), false)
                .addTemporalMarker(0.50, 0.0, () -> {
                    wobbleWrapper.openArm();
                })
                .lineToLinearHeading(new Pose2d(40.0, -26.0 + 5, Math.toRadians(180.0))).build();

        collectB = drivetrain.trajectoryBuilder(toZoneB.end(), false)
                .splineTo(new Vector2d(-30.0, -35.0 - 6), Math.toRadians(181.0)).build();

        toShooting2B = drivetrain.trajectoryBuilder(collectB.end(), true)
                .lineToLinearHeading(new Pose2d(-12.0, -35.0, Math.toRadians(195.0))).build();

        parkB = drivetrain.trajectoryBuilder(toShooting2B.end(), true)
                .lineToLinearHeading(new Pose2d(15.0, 10.0, Math.toRadians(180.0))).build();
    }

    Trajectory toZoneB, collectB, toShooting2B, parkB;


    void buildPathsFour() {
        toZoneC = drivetrain.trajectoryBuilder(toShooting.end(), false)
                .addTemporalMarker(0.50, 0.0, () -> {
                    wobbleWrapper.openArm();
                })
                .lineToLinearHeading(new Pose2d(60.0, -45.0, Math.toRadians(180.0))).build();
        collectC = drivetrain.trajectoryBuilder(toZoneC.end(), false)
                .splineTo(new Vector2d(-10.0, -35.0 - 5), Math.toRadians(180.0)).build();

        forwardC = drivetrain.trajectoryBuilder(collectC.end(), false)
                .forward(30.0, setMaxVelocity(15.0), new ProfileAccelerationConstraint(15.0)).build();

        toShooting2C = drivetrain.trajectoryBuilder(forwardC.end(),true)
                .lineToLinearHeading(new Pose2d(-12.0, -35.0, Math.toRadians(194.0))).build();

        parkC = drivetrain.trajectoryBuilder(toShooting2C.end(), true)
                .lineToLinearHeading(new Pose2d(15.0, 10.0, Math.toRadians(180.0))).build();
    }

    Trajectory toZoneC, collectC, forwardC, toShooting2C, parkC;

    MinVelocityConstraint setMaxVelocity(double maxVel) {
        return (new MinVelocityConstraint(Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL)
                , new MecanumVelocityConstraint(maxVel, DriveConstants.TRACK_WIDTH))));
    }

    public void initWebcam() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
    }
}
