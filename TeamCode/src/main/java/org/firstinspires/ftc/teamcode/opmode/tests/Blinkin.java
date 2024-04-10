package org.firstinspires.ftc.teamcode.opmode.tests;

import android.util.Log;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Config
public class Blinkin extends OpMode {
    public static RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
    private RevBlinkinLedDriver blinkin;

    @Override
    public void init() {
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
    }

    @Override
    public void loop() {
        blinkin.setPattern(pattern);
        if (gamepad1.dpad_down) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        }
        if (gamepad1.dpad_up) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
        }
        if (gamepad1.dpad_left) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_FAST;
        }
    }
}
