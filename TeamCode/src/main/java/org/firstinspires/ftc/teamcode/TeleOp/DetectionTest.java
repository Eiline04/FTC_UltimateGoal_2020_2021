package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RingDetector.AdvancedCameraThread;
import org.firstinspires.ftc.teamcode.RingDetector.CameraThread;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;


@TeleOp()
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
            if(rectHeight < 5) {
                telemetry.addLine("ZERO");
                telemetry.update();
                sleep(100);
                continue;
            }

            if(rectHeight > AdvancedCameraThread.ONE_HEIGHT && rectHeight < AdvancedCameraThread.FOUR_HEIGHT) {
                telemetry.addLine("ONE");
                telemetry.update();
                sleep(100);
                continue;
            }

            telemetry.addLine("FOUR");
            telemetry.update();
            sleep(100);
        }
    }

    public void initWebcam() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
    }
}
