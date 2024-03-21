package org.firstinspires.ftc.teamcode.auto.CV;

import android.graphics.Canvas;
import android.util.Log;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

@Config
public class ZoneDetectionProcessorRight implements VisionProcessor {
    private boolean red;
    private boolean right;
    private int zone = 3;

    public static int boxHeight = 500;
    public static int x1 = 550;
    public static int x2 = 1100;
    public static int x3 = 1500;
    public static boolean threshhold = true;

    public ZoneDetectionProcessorRight(boolean red, boolean c) {
        this.red = red;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        Log.d("asd", "entered");

        Mat ycrcb = new Mat();
        Imgproc.cvtColor(input, ycrcb, Imgproc.COLOR_RGB2YCrCb);

        Scalar blueLowThresh = new Scalar(0, 0, 132);
        Scalar blueHighThresh = new Scalar(255, 129, 255);
        Scalar redLowThresh = new Scalar(0, 151.6, 75);
        Scalar redHighThresh = new Scalar(255, 255, 255);

        if (red) {
            Core.inRange(ycrcb, redLowThresh, redHighThresh, ycrcb);
        } else {
            Core.inRange(ycrcb, blueLowThresh, blueHighThresh, ycrcb);
        }
        //
//         Split the image into three vertical quadrants
        int height = boxHeight;
        int width = ycrcb.cols();
        int quadrantWidth = width / 3;
        Scalar BLACK = new Scalar(0, 0, 0);
        Scalar BLUE = new Scalar(0, 0, 255);
        Imgproc.line(input, new Point(x1, 0), new Point(x1, 1080), BLUE, 8);
        Imgproc.line(input, new Point(x2, 0), new Point(x2, 1080), BLUE, 8);
        Imgproc.line(input, new Point(x3, 0), new Point(x3, 1080), BLUE, 8);


        int leftQuadrantWidth;
        int centerQuadrantWidth;
        int rightQuadrantWidth;
        if (right) {
        } else {

        }
        Rect left = new Rect(0, 1080-height, x1, height);

        Rect center = new Rect(x1, 1080-height, x2-x1, height);
        Rect right = new Rect(x2, 1080-height, x3-x2, height);

        // Extract the quadrants
        Mat quadrant1 = new Mat(ycrcb, left);
        Mat quadrant2 = new Mat(ycrcb, center);
        Mat quadrant3 = new Mat(ycrcb, right);

        // Calculate the average blue value for each quadrant
        Scalar blue1 = Core.mean(quadrant1);
        Scalar blue2 = Core.mean(quadrant2);
        Scalar blue3 = Core.mean(quadrant3);


        Imgproc.rectangle(input, left, BLACK, 6);
        Imgproc.rectangle(input, center, BLACK, 6);
        Imgproc.rectangle(input, right, BLACK, 6);


        // Find which quadrant has the most blue
        int blueQuadrant = 1;
        double maxBlueValue = blue1.val[0];

        if (blue2.val[0] > maxBlueValue) {
            blueQuadrant = 2;
            maxBlueValue = blue2.val[0];
        }

        if (blue3.val[0] > maxBlueValue) {
            blueQuadrant = 3;
        }

        zone = blueQuadrant;

        // Release the Mat objects created for quadrants
        quadrant1.release();
        quadrant2.release();
        quadrant3.release();
        if (threshhold) {
            return ycrcb;
        } else {
            ycrcb.release();
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