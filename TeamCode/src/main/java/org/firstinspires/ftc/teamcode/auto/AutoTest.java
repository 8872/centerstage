package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.auto.util.AutoBaseOpmode;

@Autonomous
public class AutoTest extends AutoBaseOpmode {
    @Override
    public void loop() {
        super.loop();
        telemetry.addData("fact:","yotam is based");
        telemetry.update();
    }
}
