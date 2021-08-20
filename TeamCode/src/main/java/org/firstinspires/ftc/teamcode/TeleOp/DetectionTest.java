package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RingDetector.AdvancedCameraThread;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import static org.firstinspires.ftc.teamcode.RingDetector.AdvancedCameraThread.RingPosition;

@TeleOp(group = "Misc")
public class DetectionTest extends LinearOpMode {

    OpenCvCamera webcam;
    AdvancedCameraThread cameraThread;

    @Override
    public void runOpMode() throws InterruptedException {
        initWebcam();
        sleep(1000);
        cameraThread = new AdvancedCameraThread(webcam);
        Thread cameraRunner = new Thread(cameraThread);
        cameraRunner.start();

        cameraThread.setState(AdvancedCameraThread.CAMERA_STATE.INIT);
        sleep(1000);
        cameraThread.setState(AdvancedCameraThread.CAMERA_STATE.STREAM);

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();
        telemetry.log().clear();

        while (opModeIsActive()) {
            double rectHeight = AdvancedCameraThread.rectHeight;
            double rectWidth = AdvancedCameraThread.rectWidth;
            RingPosition position = AdvancedCameraThread.getResult(rectHeight, rectWidth);

            telemetry.addData("Result:", position);
            telemetry.addData("Width", rectWidth);
            telemetry.addData("Height", rectHeight);
            telemetry.addData("Current FPS", webcam.getFps());
            telemetry.addData("Max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();

            sleep(100);
        }
    }

    public void initWebcam() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
    }
}
