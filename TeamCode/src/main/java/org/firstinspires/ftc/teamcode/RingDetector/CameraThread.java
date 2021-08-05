package org.firstinspires.ftc.teamcode.RingDetector;

import org.firstinspires.ftc.teamcode.Auto.Auto1_RED;
import org.firstinspires.ftc.teamcode.Auto.Auto2_RED;
import org.firstinspires.ftc.teamcode.Auto.Auto3_RED;
import org.firstinspires.ftc.teamcode.Auto.Auto4_RED;
import org.firstinspires.ftc.teamcode.Auto.AutoRemote;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 *  Thread that handles the webcam init and streaming for RingDeterminationPipeline
 */

public class CameraThread implements Runnable {
    static final Point regionTopLeftCorner = new Point(230, 140);

    public enum CAMERA_STATE {
        NULL,
        INIT,
        STREAM,
        DETECT,
        KILL
    }

    private OpenCvCamera camera;
    private volatile CAMERA_STATE state;
    private boolean active;
    private static volatile boolean kill = false;

    public CameraThread(OpenCvCamera camera) {
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
                    camera.setPipeline(new RingDeterminationPipeline());
                }

                if (state == CAMERA_STATE.STREAM) {
                    camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                }

                if (state == CAMERA_STATE.DETECT) {
                    AutoRemote.ringPosition = RingDeterminationPipeline.position;
                    Auto1_RED.ringPosition = RingDeterminationPipeline.position;
                    Auto2_RED.ringPosition = RingDeterminationPipeline.position;
                    Auto3_RED.ringPosition = RingDeterminationPipeline.position;
                    Auto4_RED.ringPosition = RingDeterminationPipeline.position;
                }

                if (state == CAMERA_STATE.KILL) {
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

    /**
     *  OpenCV pipeline that detects the rings by averaging out
     *  the pixels in an area and comparing it to a predetermined value.
     *  Original code courtesy of Wizards.exe, modified by Vlad Chira.
     */

    public static class RingDeterminationPipeline extends OpenCvPipeline {

        // Volatile since accessed by OpMode thread w/o synchronization
        public static volatile RingPosition position = RingPosition.FOUR;

        public enum RingPosition {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */

        static final int REGION_WIDTH = 60;
        static final int REGION_HEIGHT = 40;

        final int FOUR_RING_THRESHOLD = 145; //140
        final int ONE_RING_THRESHOLD = 135; //120

        Point regionPointA = new Point(
                regionTopLeftCorner.x,
                regionTopLeftCorner.y);
        Point regionPointB = new Point(
                regionTopLeftCorner.x + REGION_WIDTH,
                regionTopLeftCorner.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat sampledRegion;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);

            sampledRegion = Cb.submat(new Rect(regionPointA, regionPointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            avg1 = (int) Core.mean(sampledRegion).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    regionPointA, // First point which defines the rectangle
                    regionPointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if (avg1 > FOUR_RING_THRESHOLD) {
                position = RingPosition.FOUR;
            } else if (avg1 > ONE_RING_THRESHOLD) {
                position = RingPosition.ONE;
            } else {
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    regionPointA, // First point which defines the rectangle
                    regionPointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis() {
            return avg1;
        }
    }
}