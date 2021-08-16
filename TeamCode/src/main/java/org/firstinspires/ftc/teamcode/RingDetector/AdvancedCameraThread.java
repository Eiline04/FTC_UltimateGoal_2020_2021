package org.firstinspires.ftc.teamcode.RingDetector;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.teamcode.RingDetector.AdvancedCameraThread.RingPipeline.RingPosition;

import java.util.ArrayList;
import java.util.List;

/**
 * Thread that handles the webcam init and streaming for RingPipeline
 */

@Config
public class AdvancedCameraThread implements Runnable {

    public static int THRESHOLD = 110;
    public static int BLUR_KERNEL_SIZE = 9;

    public static double ONE_HEIGHT = 10;
    public static double FOUR_HEIGHT = 25;

    public enum CAMERA_STATE {
        NULL,
        INIT,
        STREAM,
        DETECT,
        KILL
    }

    private final OpenCvCamera camera;
    private volatile CAMERA_STATE state;
    private boolean active;
    private static volatile boolean kill = false;

    public AdvancedCameraThread(OpenCvCamera camera) {
        this.camera = camera;
        kill = false;
        state = CAMERA_STATE.NULL;
    }

    @Override
    public void run() {
        while (!kill) {
            if (active) {
                if (state == CAMERA_STATE.INIT) {
                    try {
                        camera.openCameraDevice();
                    } catch (OpenCvCameraException e) {
                        e.printStackTrace();
                    }
                    camera.setPipeline(new RingPipeline());
                }

                if (state == CAMERA_STATE.STREAM) {
                    camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                }

                if (state == CAMERA_STATE.KILL) {
                    camera.closeCameraDevice();
                    killThread();
                }
                active = false;
            }
        }
    }

    private static void killThread() {
        kill = true;
    }

    public void setState(CAMERA_STATE state) {
        this.state = state;
        this.active = true;
    }

    public static RingPosition getResult(double rectHeight, double rectWidth) {
        if (rectWidth < 15 || rectWidth > 70)
            return RingPosition.NONE;
        if (rectHeight < ONE_HEIGHT)
            return RingPosition.NONE;

        if (rectHeight > ONE_HEIGHT && rectHeight < FOUR_HEIGHT)
            return RingPosition.ONE;
        return RingPosition.FOUR;
    }

    public boolean isKilled() {
        return kill;
    }


    /**
     * A slightly more advanced Stage-Switching OpenCV pipeline that detects clusters of orange pixels
     * and calculates their height.
     */
    public static class RingPipeline extends OpenCvPipeline {

        public enum RingPosition {
            FOUR,
            ONE,
            NONE
        }

        public static volatile double rectHeight = 0;
        public static volatile double rectWidth = 0;

        Mat YCbCr = new Mat();
        Mat medianBlur = new Mat();
        Mat Cb = new Mat();
        Mat CbInv = new Mat();
        Mat thresholdMat = new Mat();
        Mat newmat = new Mat();
        MatOfPoint m = new MatOfPoint();
        Rect ring = new Rect();
        private List<MatOfPoint> contoursList = new ArrayList<>();
        private List<Mat> splitMat = new ArrayList<>();
        Scalar scalar = new Scalar(0, 255, 0);

        double maxArea = 0, currentArea;
        int i;

        @Override
        public void init(Mat mat) {

        }

        @Override
        public Mat processFrame(Mat input) {

            //convert to YCbCr
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);

            //blur it a bit
            Imgproc.medianBlur(YCbCr, medianBlur, BLUR_KERNEL_SIZE);

            //extract Cb channel and invert it
            Core.split(medianBlur, splitMat);
            Cb = splitMat.get(1);
            Core.bitwise_not(Cb, CbInv);

            Imgproc.threshold(CbInv, thresholdMat, THRESHOLD, 255, Imgproc.THRESH_BINARY_INV);
            Imgproc.findContours(thresholdMat, contoursList, newmat, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            //find the biggest orange spot
            ring.set(null);
            if (contoursList.size() > 0) {
                maxArea = 0;
                for (i = 0; i < contoursList.size(); i++) {
                    m = contoursList.get(i);
                    currentArea = Imgproc.contourArea(m);
                    if (currentArea > maxArea) {
                        maxArea = currentArea;
                        ring = Imgproc.boundingRect(m);
                    }
                }
            }
            rectHeight = ring.height;
            rectWidth = ring.width;
            Imgproc.rectangle(input, ring, scalar, 5);

            contoursList.clear();
            splitMat.clear();
            return input;
        }
    }

}