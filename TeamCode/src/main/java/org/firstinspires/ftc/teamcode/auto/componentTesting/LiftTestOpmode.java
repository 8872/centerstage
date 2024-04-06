package org.firstinspires.ftc.teamcode.auto.componentTesting;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.auto.util.AutoBaseOpmode;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

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
        liftSubsystem.periodic();
        if(gamepad1.a){
            schedule(liftSubsystem.goTo(LiftSubsystem.LOW));
            Log.d("asd", "i ran");
        }
    }
}
