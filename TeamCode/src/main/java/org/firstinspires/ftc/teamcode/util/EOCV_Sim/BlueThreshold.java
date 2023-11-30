package org.firstinspires.ftc.teamcode.util.EOCV_Sim;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BlueThreshold extends OpenCvPipeline {

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2YCrCb);

        return input;
    }
}