package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Config
public class TestLauncherOpMode extends OpMode {
    private SimpleServo release;
    private SimpleServo height;
    public static double heightInitialPos = 0.7;
    public static double heightFinalPos = 0.5;
    public static double releaseInitialPos = 0.6;
    public static double releaseFinalPos = 0.7;

    @Override
    public void init() {
        release = new SimpleServo(hardwareMap, "launch", 0,255);
        height = new SimpleServo(hardwareMap, "launcherServo", 0,255);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            release.setPosition(releaseInitialPos);
        }

        if (gamepad1.b) {
            release.setPosition(releaseFinalPos);
        }

        if (gamepad1.x) {
            height.setPosition(heightInitialPos);
        }

        if (gamepad1.y) {
            height.setPosition(heightFinalPos);
        }
    }
}
