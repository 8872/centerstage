package org.firstinspires.ftc.teamcode.auto.componentTesting;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.auto.util.AutoBaseOpmode;
import org.firstinspires.ftc.teamcode.subsystem.LiftSys;

@TeleOp
@Disabled
public class LiftTestOpmode extends AutoBaseOpmode {
    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        super.loop();
        liftSys.periodic();
        if(gamepad1.a){
            schedule(liftSys.goTo(LiftSys.LOW));
            Log.d("asd", "i ran");
        }
    }
}
