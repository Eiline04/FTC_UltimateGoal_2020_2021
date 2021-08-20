package org.firstinspires.ftc.teamcode.Auto;

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
import org.firstinspires.ftc.teamcode.Wrappers.DasPositions;
import org.firstinspires.ftc.teamcode.Wrappers.Intake;
import org.firstinspires.ftc.teamcode.Wrappers.LauncherWrapper;
import org.firstinspires.ftc.teamcode.Wrappers.WobbleWrapper;
import org.firstinspires.ftc.teamcode.RingDetector.AdvancedCameraThread;
import org.firstinspires.ftc.teamcode.Roadrunner.DriveConstants;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import static org.firstinspires.ftc.teamcode.RingDetector.AdvancedCameraThread.RingPosition;

import java.util.Arrays;

@Autonomous(name = "Auto Remote")
@Disabled
@Deprecated
public class AutoRemote extends LinearOpMode {

    public static volatile RingPosition ringPosition;
    OpenCvCamera webcam;
    AdvancedCameraThread cameraThread;

    DasPositions positionDAS;

    LauncherWrapper launcher;
    WobbleWrapper wobbleWrapper;
    Hardware robot;
    Intake intake;

    DasPositions dasPositions;

    MecanumDrive drivetrain;
    Pose2d startPose = new Pose2d(-62.95, -23.62, Math.toRadians(180.0));

    private final int launchSleepTime = 300;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Hardware();
        robot.init(hardwareMap);
        launcher = new LauncherWrapper(robot.launcherTop, robot.launcherBottom, robot.launchServo, robot.ringStopper);
        launcher.closeStopper();
        launcher.setPIDFCoeff(new PIDFCoefficients(55, 0, 0, 11.5));
        wobbleWrapper = new WobbleWrapper(robot.gripperServo, robot.armServo, robot.wobbleRelease);
        intake = new Intake(robot.staticIntake, robot.mobileIntake, robot.mopStanga, robot.mopDreapta);

        positionDAS = new DasPositions(robot.servoDAS);

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

        robot.enableBulkDataPolling();

        dasPositions = new DasPositions(robot.servoDAS);
        dasPositions.startDAS();

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;



        telemetry.addData("Result", ringPosition);
        telemetry.update();

        drivetrain = new MecanumDrive(hardwareMap);
        drivetrain.setPoseEstimate(startPose);

        cameraThread.setState(AdvancedCameraThread.CAMERA_STATE.KILL);

        //-------------------------------NO RINGS---------------------------------------
        if (ringPosition == RingPosition.NONE) {
            buildPathsNone();

            drivetrain.followTrajectory(toZoneA); //go to zone A. Arm is lowered with temporal marker
            sleep(100);
            wobbleWrapper.detachGrip();
            sleep(200);
            wobbleWrapper.setArmPosition(0.5f);

            launcher.setVelocity(620, AngleUnit.DEGREES);
            intake.startIntake();
            drivetrain.followTrajectory(toShootingA);
            launcher.launchOneRing();
            sleep(launchSleepTime);
            launcher.launchOneRing();
            sleep(launchSleepTime);
            launcher.launchOneRing();
            sleep(launchSleepTime);
            launcher.launchOneRing();
            intake.stopIntake();

            drivetrain.followTrajectory(toSecondWobbleA); //arm is lowered with temporal marker
            launcher.setPower(0);
            sleep(100);
            drivetrain.followTrajectory(strafeToWobble_A);
            sleep(500);
            wobbleWrapper.attachGrip();
            sleep(500);
            wobbleWrapper.setArmPosition(0.8f);

            drivetrain.followTrajectory(toZoneA_2); //go back to Zone A for second wobble
            wobbleWrapper.detachGrip();
            sleep(200);
            wobbleWrapper.setArmPosition(0.5f);

            sleep(300);
            wobbleWrapper.attachGrip();
            sleep(300);
            //wobbleWrapper.setArmPosition(0); //retract arm
            wobbleWrapper.closeArm();

            drivetrain.followTrajectory(drivetrain.trajectoryBuilder(toZoneA_2.end()).strafeRight(23).build()); //strafe away to park
            sleep(100);
        }

        //-------------------------------ONE RING--------------------------------
        if (ringPosition == RingPosition.ONE) {
            buildPathsOne();

            drivetrain.followTrajectory(toZoneB); //go to zone b
            sleep(100);
            wobbleWrapper.detachGrip();
            sleep(200);
            wobbleWrapper.setArmPosition(0.5f);

            launcher.setVelocity(615, AngleUnit.DEGREES);
            intake.startIntake();
            drivetrain.followTrajectory(toShootingB);
            sleep(200);
            launcher.launchOneRing();  //launch preloaded
            sleep(launchSleepTime);
            launcher.launchOneRing();
            sleep(launchSleepTime);
            launcher.launchOneRing();
            sleep(launchSleepTime);
            launcher.launchOneRing();

            launcher.setVelocity(615, AngleUnit.DEGREES);
            drivetrain.followTrajectory(toMiddleRingB);
            launcher.launchOneRing();
            sleep(launchSleepTime);
            launcher.launchOneRing();
            sleep(300);

            drivetrain.followTrajectory(toSecondWobbleB);
            intake.stopIntake();
            launcher.setPower(0);
            drivetrain.followTrajectory(strafeToWobble_B);
            sleep(500);
            wobbleWrapper.attachGrip();
            sleep(500);
            //wobbleWrapper.setArmPosition(0.9f);
            wobbleWrapper.openArm();
            sleep(200);

            drivetrain.followTrajectory(toZoneB_2);
            wobbleWrapper.detachGrip();
            sleep(200);
            wobbleWrapper.setArmPosition(0.6f);
            sleep(500);
            wobbleWrapper.attachGrip();
            sleep(200);
            //wobbleWrapper.setArmPosition(0); //retract arm
            wobbleWrapper.closeArm();

            drivetrain.followTrajectory(drivetrain.trajectoryBuilder(toZoneB_2.end(), false).forward(15.74).build());
            sleep(100);
        }

        //-------------------------------FOUR RING--------------------------------
        if (ringPosition == RingPosition.FOUR) {
            buildPathsFour();

            drivetrain.followTrajectory(toZoneC);
            wobbleWrapper.detachGrip();
            sleep(200);
            wobbleWrapper.setArmPosition(0.6f);

            launcher.setVelocity(615 + 40, AngleUnit.DEGREES);
            dasPositions.setPositionDAS(0.76);

            drivetrain.followTrajectory(toStackC);

            sleep(200);
            launcher.launchOneRing();  //launch preloaded
            sleep(launchSleepTime);
            launcher.launchOneRing();
            sleep(launchSleepTime);
            launcher.launchOneRing();
            sleep(launchSleepTime);
            launcher.launchOneRing();

            intake.startIntake();
            wobbleWrapper.setArmPosition(0.5f);

            launcher.setVelocity(615 + 45, AngleUnit.DEGREES);
            drivetrain.followTrajectory(forwardC);
            drivetrain.followTrajectory(backwardsC);
            drivetrain.followTrajectory(forwardsC_2);

            sleep(200);
            launcher.launchOneRing();  //launch
            sleep(launchSleepTime);
            launcher.launchOneRing();
            sleep(launchSleepTime);
            launcher.launchOneRing();
            sleep(launchSleepTime);
            launcher.launchOneRing();

            wobbleWrapper.openArm();
            drivetrain.followTrajectory(toSecondWobbleC);
            sleep(500);
            wobbleWrapper.attachGrip();
            sleep(300);
            drivetrain.followTrajectory(toZoneC_2);
            wobbleWrapper.detachGrip();
            sleep(500);
            drivetrain.followTrajectory(toParkC);
        }
        drivetrain.updatePoseEstimate();
        wobbleWrapper.attachGrip();
        sleep(100);
    }

    void rotateToRight(MecanumDrive drivetrain, double radians, double power) {
        robot.frontRightWheel.setPower(-power);
        robot.frontLeftWheel.setPower(power);
        robot.backRightWheel.setPower(-power);
        robot.backLeftWheel.setPower(power);

        drivetrain.updatePoseEstimate();
        while (drivetrain.getPoseEstimate().getHeading() > radians && opModeIsActive()) {
            drivetrain.updatePoseEstimate();
        }

        robot.frontRightWheel.setPower(0);
        robot.frontLeftWheel.setPower(0);
        robot.backRightWheel.setPower(0);
        robot.backLeftWheel.setPower(0);
    }

    void buildPathsNone() {
        toZoneA = drivetrain.trajectoryBuilder(startPose, true)
                .lineToConstantHeading(new Vector2d(17.67, -53.89)) //go to zone A
                .addTemporalMarker(0.30, 0.0, () -> {
                    //wobbleWrapper.setArmPosition(1f); //lower the arm
                    wobbleWrapper.openArm();
                }).build();


        toShootingA = drivetrain.trajectoryBuilder(toZoneA.end(), false)
                .lineToLinearHeading(new Pose2d(-20.76, -36.0, Math.toRadians(175.0)), setMaxVelocity(35.0), new ProfileAccelerationConstraint(35.0))
                .build();

        toSecondWobbleA = drivetrain.trajectoryBuilder(toShootingA.end(), false)
                .lineToLinearHeading(new Pose2d(-48.38, -31.45, Math.toRadians(180.0)), setMaxVelocity(25.0), new ProfileAccelerationConstraint(20.0)) //go to wobble goal
                .addTemporalMarker(0.40, 0.0, () -> {
                    // This will run 40% of the way through the path
                    //wobbleWrapper.setArmPosition(1);
                    wobbleWrapper.openArm();
                })
                .build();

        strafeToWobble_A = drivetrain.trajectoryBuilder(toSecondWobbleA.end(), true)
                .strafeLeft(3.93, setMaxVelocity(30.0), new ProfileAccelerationConstraint(25.0)).build();

        toZoneA_2 = drivetrain.trajectoryBuilder(strafeToWobble_A.end(), true)
                .lineToConstantHeading(new Vector2d(10.98, -46.02))
                .addTemporalMarker(0.30, 0.0, () -> {
                    // This will run 70% of the way through the path
                    //wobbleWrapper.setArmPosition(1);
                    wobbleWrapper.openArm();
                })
                .build();
    }

    Trajectory toZoneA, toShootingA, toSecondWobbleA, strafeToWobble_A, toZoneA_2;


    void buildPathsOne() {
        toZoneB = drivetrain.trajectoryBuilder(startPose, true)
                .splineToSplineHeading(new Pose2d(-20.0, -20.0, Math.toRadians(180.0)), 0.0)
                .splineToSplineHeading(new Pose2d(42.08, -29.88, Math.toRadians(180.0)), 0.0) //to zone B
                .addTemporalMarker(0.30, 0.0, () -> {
                    // This will run 70% of the way through the path
                    //wobbleWrapper.setArmPosition(1f);
                    wobbleWrapper.openArm();
                }).build();

        toShootingB = drivetrain.trajectoryBuilder(toZoneB.end(), false)
                .lineToLinearHeading(new Pose2d(4.76 - 7.0, -39.0, Math.toRadians(176.05)), setMaxVelocity(35.0), new ProfileAccelerationConstraint(35.0)).build();

        toMiddleRingB = drivetrain.trajectoryBuilder(toShootingB.end(), false)
                .forward(25.0).build();

        toSecondWobbleB = drivetrain.trajectoryBuilder(toMiddleRingB.end(), false)
                .addTemporalMarker(0.40, 0.0, () -> {
                    // This will run 40% of the way through the path
                    //wobbleWrapper.setArmPosition(1f);
                    wobbleWrapper.openArm();
                })
                .lineToLinearHeading(new Pose2d(-48.38, -31.45, Math.toRadians(180.0)), setMaxVelocity(30.0), new ProfileAccelerationConstraint(25.0)).build();

        strafeToWobble_B = drivetrain.trajectoryBuilder(toSecondWobbleB.end(), true)
                .strafeLeft(3.93 + 2.16 + 1.5, setMaxVelocity(30.0), new ProfileAccelerationConstraint(20.0)).build();

        toZoneB_2 = drivetrain.trajectoryBuilder(strafeToWobble_B.end(), true)
                .splineToSplineHeading(new Pose2d(32.63, -19.64, Math.toRadians(180.0)), 0.0).build();
    }

    Trajectory toZoneB, toShootingB, toMiddleRingB, toSecondWobbleB, strafeToWobble_B, toZoneB_2;


    void buildPathsFour() {
        toZoneC = drivetrain.trajectoryBuilder(startPose, true)
                .splineToSplineHeading(new Pose2d(-8.0, -20.0, Math.toRadians(180.0)), 0.0)
                .splineToSplineHeading(new Pose2d(62.5, -53.5 + 0.7, Math.toRadians(180.0)), 0.0)
                .addTemporalMarker(0.20, 0.0, () -> {
                    // This will run 60% of the way through the path
                    //wobbleWrapper.setArmPosition(1f);
                    wobbleWrapper.openArm();
                })
                .build();

        toStackC = drivetrain.trajectoryBuilder(toZoneC.end(), false)
                .lineToConstantHeading(new Vector2d(1.30, -36.0 + 4), setMaxVelocity(35.0), new ProfileAccelerationConstraint(35.0)).build();


        forwardC = drivetrain.trajectoryBuilder(toStackC.end(), false)
                .addTemporalMarker(0.7, 0.0, () -> {
                    launcher.launchOneRing();
                })
                .addTemporalMarker(0.9, 0.0, ()-> {
                    launcher.launchOneRing();
                })
                .forward(15.74 + 11.8 + 10.0, setMaxVelocity(20.0), new ProfileAccelerationConstraint(20.0)).build();

        backwardsC = drivetrain.trajectoryBuilder(forwardC.end(), true)
                .back(8.0, setMaxVelocity(15.0), new ProfileAccelerationConstraint(15.0)).build();

        forwardsC_2 = drivetrain.trajectoryBuilder(backwardsC.end(), false)
                .forward(24.0, setMaxVelocity(20.0), new ProfileAccelerationConstraint(15.0)).build();

        toSecondWobbleC = drivetrain.trajectoryBuilder(forwardsC_2.end(), false)
                .lineToLinearHeading(new Pose2d(-46.5 + 2.5, -36.6, Math.toRadians(173.0)), setMaxVelocity(15.0), new ProfileAccelerationConstraint(20.0))
                .build();

        toZoneC_2 = drivetrain.trajectoryBuilder(toSecondWobbleC.end(), true)
                .addTemporalMarker(0.30, 0.0, () -> {
                    launcher.setPower(0);
                })
                .splineToSplineHeading(new Pose2d(54.5 + 5.0, -49.0 + 6.0, Math.toRadians(173.0)), 0.0).build();

        toParkC = drivetrain.trajectoryBuilder(toZoneC_2.end(), false)
                .forward(38.0).build();
    }

    Trajectory toZoneC, toStackC, forwardC, backwardsC, forwardsC_2, toSecondWobbleC, toZoneC_2, toParkC;

    MinVelocityConstraint setMaxVelocity(double maxVel) {
        return (new MinVelocityConstraint(Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL)
                , new MecanumVelocityConstraint(maxVel, DriveConstants.TRACK_WIDTH))));
    }

    public void initWebcam() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
    }
}


