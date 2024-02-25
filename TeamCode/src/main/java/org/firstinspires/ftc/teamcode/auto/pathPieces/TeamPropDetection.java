package org.firstinspires.ftc.teamcode.auto.pathPieces;

import android.util.Log;
import android.util.Size;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.CV.ZoneDetectionProcessor;
import org.firstinspires.ftc.teamcode.auto.util.AutoBaseOpmode;
import org.firstinspires.ftc.teamcode.util.commands.DelayedCommand;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@TeleOp(name="Team Prop Detection", group = "ZZZ")
public class TeamPropDetection extends AutoBaseOpmode {

    private boolean red;
    private boolean right;
    private VisionPortal portal;
    private ZoneDetectionProcessor processor;

    public TeamPropDetection(boolean red, boolean right){
        this.red = red;
        this.right = right;
    }

    @Override
    public void init(){
        super.init();
        Log.d("asd", "finished super.init");
        processor = new ZoneDetectionProcessor(red,right);
        portal = new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(1280, 720))
                .setAutoStopLiveView(true)
                .enableLiveView(true)
                .build();
        Log.d("asd", "opened vision portal");
    }

    @Override
    public void loop() {
        super.loop();
    }

    @Override
    public void stop() {
        portal.close();
    }

    public int getZone(){
        return processor.getZone();
    }
}
