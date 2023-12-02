package org.firstinspires.ftc.teamcode.auto.pipelines;

import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;


public class BlueZoneDetectionPipeline extends OpenCvPipeline {

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2YCrCb);

        Scalar lowThresh = new Scalar(0, 0, 158.7);
        Scalar highThresh = new Scalar(255, 180, 95);

        Core.inRange(input, lowThresh, highThresh, input);

        // Split the image into three vertical quadrants
        int height = input.rows();
        int width = input.cols();
        int quadrantWidth = width / 3;
        Rect left = new Rect(0, 0, quadrantWidth, height);
        Rect center = new Rect(quadrantWidth, 0, quadrantWidth, height);
        Rect right = new Rect(2 * quadrantWidth, 0, width - 2 * quadrantWidth, height);

        // Extract the quadrants
        Mat quadrant1 = new Mat(input, left);
        Mat quadrant2 = new Mat(input, center);
        Mat quadrant3 = new Mat(input, right);

        // Calculate the average blue value for each quadrant
        Scalar blue1 = Core.mean(quadrant1);
        Scalar blue2 = Core.mean(quadrant2);
        Scalar blue3 = Core.mean(quadrant3);

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
        telemetry.addData("Quadrant", blueQuadrant);
        telemetry.update();

        // Release the Mat objects created for quadrants
        quadrant1.release();
        quadrant2.release();
        quadrant3.release();

        // Return the masked input Mat (or modify based on your requirements)
        return input;
    }
}
