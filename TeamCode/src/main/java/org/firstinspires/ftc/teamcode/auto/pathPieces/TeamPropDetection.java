package org.firstinspires.ftc.teamcode.auto.pathPieces;

import android.util.Log;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.auto.util.AutoBaseOpmode;

@Config
@TeleOp(name = "Team Prop Detection", group = "ZZZ")
public class TeamPropDetection extends AutoBaseOpmode {

    protected TeamPropDetection(boolean red, boolean right) {
//        super(red, right);
    }

    @Override
    public void init() {
        super.init();
        Log.d("asd", "finished super.init");
        Log.d("asd", "opened vision portal");
    }

    @Override
    public void loop() {
        super.loop();

        telemetry.addData("processor zone", getZone());
        telemetry.update();
    }

    @Override
    public void stop() {
        portal.close();
    }

    public int getZone() {
        return processor.getZone();
    }
}
