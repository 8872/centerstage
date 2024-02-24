//package org.firstinspires.ftc.teamcode.auto.pipelines;
//
//import android.graphics.Bitmap;
//import android.graphics.Canvas;
//import android.util.Log;
//import com.acmerobotics.dashboard.config.Config;
//import org.firstinspires.ftc.robotcore.external.function.Consumer;
//import org.firstinspires.ftc.robotcore.external.function.Continuation;
//import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
//import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
//import org.firstinspires.ftc.vision.VisionProcessor;
//import org.opencv.android.Utils;
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//
//import java.util.concurrent.atomic.AtomicReference;
//
//@Config
//public class ZoneDetectionProcessor implements VisionProcessor, CameraStreamSource {
//
//    private boolean red;
//    private boolean right;
//    private int zone = 0;
//
//    public ZoneDetectionProcessor(boolean red, boolean right){
//        this.red = red;
//        this.right = right;
//    }
//
//    private final AtomicReference<Bitmap> lastFrame =
//            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
//
//    @Override
//    public void init(int width, int height, CameraCalibration calibration) {
//        Log.d("asd", "init");
//        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
//    }
//
//    @Override
//    public Object processFrame(Mat input, long captureTimeNanos) {
//        Log.d("asd", "entered");
//
//        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2YCrCb);
//
//        Scalar blueLowThresh = new Scalar(0, 0, 159);
//        Scalar blueHighThresh = new Scalar(255, 255, 255);
//        Scalar redLowThresh = new Scalar(0, 170, 0);
//        Scalar redHighThresh = new Scalar(255, 255, 255);
//
//        if(red){
//            Core.inRange(input, redLowThresh, redHighThresh, input);
//        }else{
//            Core.inRange(input, blueLowThresh, blueHighThresh, input);
//
//        }
//
//        // Split the image into three vertical quadrants
//        int height = (int) (input.rows()*0.7);
//        int width = input.cols();
//        int leftQuadrantWidth;
//        int centerQuadrantWidth;
//        int rightQuadrantWidth;
//        if(right){
//        }else{
//
//        }
//        Rect left = new Rect(0, 0, quadrantWidth, height);
//        Rect center = new Rect(quadrantWidth, 0, quadrantWidth, height);
//        Rect right = new Rect(2 * quadrantWidth, 0, width - 2 * quadrantWidth, height);
//
//        // Extract the quadrants
//        Mat quadrant1 = new Mat(input, left);
//        Mat quadrant2 = new Mat(input, center);
//        Mat quadrant3 = new Mat(input, right);
//
//        // Calculate the average blue value for each quadrant
//        Scalar blue1 = Core.mean(quadrant1);
//        Scalar blue2 = Core.mean(quadrant2);
//        Scalar blue3 = Core.mean(quadrant3);
//
//        // Find which quadrant has the most blue
//        int blueQuadrant = 1;
//        double maxBlueValue = blue1.val[0];
//
//        if (blue2.val[0] > maxBlueValue) {
//            blueQuadrant = 2;
//            maxBlueValue = blue2.val[0];
//        }
//
//        if (blue3.val[0] > maxBlueValue) {
//            blueQuadrant = 3;
//        }
//
//        zone = blueQuadrant;
//
//        // Release the Mat objects created for quadrants
//        quadrant1.release();
//        quadrant2.release();
//        quadrant3.release();
//
//        Bitmap b = Bitmap.createBitmap(input.width(), input.height(), Bitmap.Config.RGB_565);
//        Utils.matToBitmap(input, b);
//        lastFrame.set(b);
//        return null;
//    }
//
//    @Override
//    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
//                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
//                            Object userContext) {
//        // do nothing
//    }
//
//    @Override
//    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
//        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
//    }
//
//    public int getZone(){
//        return zone;
//    }
//}