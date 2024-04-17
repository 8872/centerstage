package org.firstinspires.ftc.teamcode.auto.CV;

import android.graphics.Canvas;
import android.util.Log;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

@Config
public class ZoneDetectionThresholder implements VisionProcessor {
    private boolean red;
    private boolean right;
    private int zone = 1;

    public static int boxHeight = 500;
    public static int x1 = 550;
    public static int x2 = 930;
    public static int x3 = 1500;
    public static boolean threshhold = true;

    public ZoneDetectionThresholder(boolean red) {
        this.red = red;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        Log.d("asd", "entered");

        Mat ycrcb = new Mat();
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);

        Scalar blueLowThresh = new Scalar(0, 43.9, 62.3);
        Scalar blueHighThresh = new Scalar(255, 255, 255); // TODO thresh
        Scalar redLowThresh = new Scalar(0, 140, 0);
        Scalar redHighThresh = new Scalar(255, 255, 255);

        if (red) {
            Core.inRange(input, redLowThresh, redHighThresh, input);
        } else {
            Core.inRange(input, blueLowThresh, blueHighThresh, input);
        }
        return input;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
        // do nothing
    }

    public int getZone() {
        return zone;
    }
}