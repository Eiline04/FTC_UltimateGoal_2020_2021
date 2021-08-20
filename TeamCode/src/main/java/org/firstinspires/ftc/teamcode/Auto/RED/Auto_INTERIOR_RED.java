package org.firstinspires.ftc.teamcode.Auto.RED;

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

import static org.firstinspires.ftc.teamcode.RingDetector.AdvancedCameraThread.RingPosition;

import java.util.Arrays;

@Autonomous(group = "RED")
public class Auto_INTERIOR_RED extends LinearOpMode {
    public static volatile RingPosition ringPosition;
    OpenCvCamera webcam;
    AdvancedCameraThread cameraThread;

    LauncherWrapper launcher;
    WobbleWrapper wobbleWrapper;
    Hardware robot;
    Intake intake;

    DasPositions dasPositions;

    MecanumDrive drivetrain;
    Pose2d startPose = new Pose2d(-62.95, -23.62, Math.toRadians(180.0));

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

        robot.enableBulkDataPolling();

        dasPositions = new DasPositions(robot.servoDAS);
        dasPositions.setPositionDAS(0.7);

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

        toShooting = drivetrain.trajectoryBuilder(startPose, true).lineToLinearHeading(new Pose2d(-12.0, -14.0, Math.toRadians(180.0))).build();

        launcher.openStopper();
        launcher.setVelocity(LauncherWrapper.shootingVelocity, AngleUnit.DEGREES);
        drivetrain.followTrajectory(toShooting);

        //-----------------ZERO---------------
        if (ringPosition == RingPosition.NONE) {
            buildPathsZero();

        }

        //------------------ONE----------------
        if (ringPosition == RingPosition.ONE) {
            buildPathsOne();

        }
        //-----------------FOUR-----------------
        if (ringPosition == RingPosition.FOUR) {
            buildPathsFour();

        }

        wobbleWrapper.attachGrip();
        wobbleWrapper.closeArm();
        sleep(300);
    }

    void buildPathsZero() {
        toZoneA = drivetrain.trajectoryBuilder(toShooting.end(), false)
                .addTemporalMarker(0.50, 0.0, () -> {
                    wobbleWrapper.openArm();
                })
                .lineToLinearHeading(new Pose2d(12.0, -50.0, Math.toRadians(180.0))).build();
    }

    Trajectory toZoneA;

    void buildPathsOne() {
        toZoneB = drivetrain.trajectoryBuilder(toShooting.end(), false)
                .addTemporalMarker(0.50, 0.0, () -> {
                    wobbleWrapper.openArm();
                })
                .lineToLinearHeading(new Pose2d(40.0, -26.0 + 5, Math.toRadians(180.0))).build();
    }

    Trajectory toZoneB;


    void buildPathsFour() {
        toZoneC = drivetrain.trajectoryBuilder(toShooting.end(), false)
                .addTemporalMarker(0.50, 0.0, () -> {
                    wobbleWrapper.openArm();
                })
                .lineToLinearHeading(new Pose2d(60.0, -45.0, Math.toRadians(180.0))).build();

    }

    Trajectory toZoneC;

    MinVelocityConstraint setMaxVelocity(double maxVel) {
        return (new MinVelocityConstraint(Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL)
                , new MecanumVelocityConstraint(maxVel, DriveConstants.TRACK_WIDTH))));
    }

    public void initWebcam() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
    }
}
