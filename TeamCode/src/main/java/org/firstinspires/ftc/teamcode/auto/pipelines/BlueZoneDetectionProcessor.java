package org.firstinspires.ftc.teamcode.auto.pipelines;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Log;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.atomic.AtomicReference;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class BlueZoneDetectionProcessor implements VisionProcessor, CameraStreamSource {

    private int zone = 0;

    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        Log.d("asd", "init");
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        Log.d("asd", "entered");

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

        zone = blueQuadrant;

        // Release the Mat objects created for quadrants
        quadrant1.release();
        quadrant2.release();
        quadrant3.release();

        Bitmap b = Bitmap.createBitmap(input.width(), input.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(input, b);
        lastFrame.set(b);
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
        // do nothing
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }

    public int getZone(){
        return zone;
    }
}