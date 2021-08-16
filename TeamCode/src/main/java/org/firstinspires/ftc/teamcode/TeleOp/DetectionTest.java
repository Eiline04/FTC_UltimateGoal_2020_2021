package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RingDetector.AdvancedCameraThread;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import static org.firstinspires.ftc.teamcode.RingDetector.AdvancedCameraThread.RingPipeline.RingPosition;

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
        sleep(2500);
        cameraThread.setState(AdvancedCameraThread.CAMERA_STATE.STREAM);
        sleep(1000);

        waitForStart();

        while (opModeIsActive()) {
            double rectHeight = AdvancedCameraThread.RingPipeline.rectHeight;
            double rectWidth = AdvancedCameraThread.RingPipeline.rectWidth;
            RingPosition position = AdvancedCameraThread.getResult(rectHeight,rectWidth);

            if(position == RingPosition.NONE) {
                telemetry.addLine("NONE");
                telemetry.addData("Width", rectWidth);
                telemetry.addData("Height", rectHeight);
                telemetry.addData("Current FPS", webcam.getFps());
                telemetry.addData("Max FPS", webcam.getCurrentPipelineMaxFps());
                telemetry.update();
                continue;
            }
            if(position == RingPosition.ONE) {
                telemetry.addLine("ONE");
                telemetry.addData("Width", rectWidth);
                telemetry.addData("Height", rectHeight);
                telemetry.addData("Current FPS", webcam.getFps());
                telemetry.addData("Max FPS", webcam.getCurrentPipelineMaxFps());
                telemetry.update();
                continue;
            }

            telemetry.addLine("FOUR");
            telemetry.addData("Width", rectWidth);
            telemetry.addData("Height", rectHeight);
            telemetry.addData("Current FPS", webcam.getFps());
            telemetry.addData("Max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();
        }
    }

    public void initWebcam() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
    }
}
