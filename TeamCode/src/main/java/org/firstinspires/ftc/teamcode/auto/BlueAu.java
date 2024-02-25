package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.auto.pathPieces.audienceStart.WhiteStackPickupAu;
import org.firstinspires.ftc.teamcode.auto.pathPieces.audienceStart.blue.BluePurplePixelAu;
import org.firstinspires.ftc.teamcode.auto.util.AutoBaseOpmode;

@Autonomous
@Config
public class BlueAu extends AutoBaseOpmode {

    public static int zone = 1;

    BluePurplePixelAu purplePixel;
    WhiteStackPickupAu whiteStack;

    @Override
    public void init(){
        super.init();
        purplePixel = new BluePurplePixelAu();
        whiteStack = new WhiteStackPickupAu();
    }

    @Override
    public void init_loop(){
        //detect team prop here
    }

    @Override
    public void start(){
        purplePixel.run(zone, false);
    }

    @Override
    public void loop(){
        if(purplePixel.isFinished()) whiteStack.run(false);
        if(whiteStack.isFinished()) telemetry.addData("Finished", true);
    }

    public void runBackground(){
        super.loop();
        purplePixel.loop();
        whiteStack.loop();
    }
}
