package org.firstinspires.ftc.teamcode.auto.CV;

import android.util.Size;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
@Config
public class TestVisionPortalLeft extends OpMode {
    private ZoneDetectionProcessorLeft processor;
    private VisionPortal portal;
    public static boolean red = true;


    @Override
    public void init() {
        processor = new ZoneDetectionProcessorLeft(red);
        portal = new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(1920, 1080))
                .setAutoStopLiveView(true)
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .build();
    }

    @Override
    public void loop() {
        telemetry.addData("zone", processor.getZone());
        telemetry.update();
    }

    @Override
    public void stop() {
        portal.close();
    }
}
