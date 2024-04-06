package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
@Disabled
public class LauncherTest extends OpMode {

    public static double staticPos = 0.2;
    public static double releasePos = 0.7;

    SimpleServo plane;
    @Override
    public void init() {
        plane = new SimpleServo(hardwareMap, "airplane", 0, 255);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            plane.setPosition(staticPos);
        } if (gamepad1.b) {
            plane.setPosition(releasePos);
        }
    }
}
