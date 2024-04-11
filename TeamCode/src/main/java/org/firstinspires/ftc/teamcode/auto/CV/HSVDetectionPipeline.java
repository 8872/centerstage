package org.firstinspires.ftc.teamcode.auto.CV;

import android.graphics.Canvas;
import android.util.Log;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

// use this one
@Config
public class HSVDetectionPipeline implements VisionProcessor {
    private final Side side;
    private int zone = 1;

    public static boolean DEBUG = true;

    public static boolean threshhold = true;

    // hsv
    public static int redThreshold = 140;
    public static int blueThreshold = 0;

    public static int redLeftX = 645;
    public static int redLeftY = 515;
    public static int redCenterX = 950;
    public static int redCenterY = 500;
    public static int redRightX = 1650;
    public static int redRightY = 500;
    public static int blueLeftX = 150; // TODO blue top left
    public static int blueLeftY = 500;
    public static int blueCenterX = 600;
    public static int blueCenterY = 500;
    public static int blueRightX = 1150;
    public static int blueRightY = 500;
    public static int rightWidth = 200;
    public static int rightHeight = 200;
    public static int leftWidth = 200;
    public static int leftHeight = 200;
    public static int centerWidth = 450;
    public static int centerHeight = 150;

    public HSVDetectionPipeline(Side side) {
        this.side = side;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        boolean isRed = side == Side.RED_CLOSE || side == Side.RED_FAR;
        Mat thresh = new Mat();
        Imgproc.cvtColor(input, thresh, Imgproc.COLOR_RGB2HSV);

        Scalar blueLowThresh = new Scalar(0, 43.9, 62.3);
        Scalar blueHighThresh = new Scalar(255, 255, 255); // TODO thresh
        Scalar redLowThresh = new Scalar(0, 140, 0);
        Scalar redHighThresh = new Scalar(255, 255, 255);

        Rect leftZoneArea, centerZoneArea, rightZoneArea;
        if (side == Side.RED_FAR || side == Side.BLUE_CLOSE) {
            leftZoneArea = new Rect(redLeftX, redLeftY, leftWidth, leftHeight);
            centerZoneArea = new Rect(redCenterX, redCenterY, centerWidth, centerHeight);
            rightZoneArea = new Rect(redRightX, redRightY, rightWidth, rightHeight);
        } else {
            leftZoneArea = new Rect(blueLeftX, blueLeftY, leftWidth, leftHeight);
            centerZoneArea = new Rect(blueCenterX, blueCenterY, centerWidth, centerHeight);
            rightZoneArea = new Rect(blueRightX, blueRightY, rightWidth, rightHeight);
        }

        if (isRed) {
            Core.inRange(thresh, redLowThresh, redHighThresh, thresh);
        } else {
            Core.inRange(thresh, blueLowThresh, blueHighThresh, thresh);
        }

        Mat leftZone = thresh.submat(leftZoneArea);
        Mat centerZone = thresh.submat(centerZoneArea);
        Mat rightZone = thresh.submat(rightZoneArea);

        if (DEBUG) {
            Imgproc.blur(input, input, new Size(5, 5));
            Imgproc.rectangle(input, leftZoneArea, new Scalar(255, 255, 255), 2);
            Imgproc.rectangle(input, centerZoneArea, new Scalar(255, 255, 255), 2);
            Imgproc.rectangle(input, rightZoneArea, new Scalar(255, 255, 255), 2);
        }

        Imgproc.blur(leftZone, leftZone, new Size(5, 5));
        Imgproc.blur(centerZone, centerZone, new Size(5, 5));
        Imgproc.blur(rightZone, rightZone, new Size(5, 5));

        // Calculate the average blue value for each quadrant
        Scalar left = Core.mean(leftZone);
        Scalar center = Core.mean(centerZone);
        Scalar right = Core.mean(rightZone);

        Log.d("asd", "left: " + left.val[0]);
        Log.d("asd", "center: " + center.val[0]);
        Log.d("asd", "right: " + right.val[0]);


        double max = Math.max(left.val[0], Math.max(right.val[0], center.val[0]));

        if (left.val[0] == max) {
            zone = 1;
            Imgproc.rectangle(input, leftZoneArea, new Scalar(255, 255, 255), 10);
        } else if (center.val[0] == max) {
            zone = 2;
            Imgproc.rectangle(input, centerZoneArea, new Scalar(255, 255, 255), 10);
        } else {
            zone = 3;
            Imgproc.rectangle(input, rightZoneArea, new Scalar(255, 255, 255), 10);
        }

        leftZone.release();
        centerZone.release();
        rightZone.release();
        if (threshhold) {
            return thresh;
        } else {
            thresh.release();
            return input;
        }
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