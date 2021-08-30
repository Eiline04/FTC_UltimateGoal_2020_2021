package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Miscellaneous.ControllerInput;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;

import java.io.File;

/**
 * Simple OpMode to test the localization of the robot. It can also log coordinates
 * to file if needed.
 */
@TeleOp(name = "Localization Test", group = "Misc")
public class LocalizationTest extends LinearOpMode {

    public boolean writeToFile = false;

    private File coordinates = AppUtil.getInstance().getSettingsFile("coordinates.txt");
    private StringBuilder stringBuilder;

    @Override
    public void runOpMode() {
        Hardware robot = new Hardware();
        robot.init(hardwareMap);
        robot.enableBulkDataPolling();

        MecanumDrive drive = new MecanumDrive(hardwareMap);
        ControllerInput controller1;

        Pose2d startPose = new Pose2d(-48.4, -63.0, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        controller1 = new ControllerInput(gamepad1);

        stringBuilder = new StringBuilder();
        waitForStart();

        while (opModeIsActive()) {
            controller1.update();

            drive.setWeightedDrivePower(new Pose2d(-controller1.left_stick_y, -controller1.left_stick_x, -controller1.right_stick_x));
            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.update();

            stringBuilder.append(poseEstimate.getX()).append(" ").append(poseEstimate.getY()).append("\n");
        }

        if (writeToFile) {
            ReadWriteFile.writeFile(coordinates, stringBuilder.toString());
        }
    }
}
