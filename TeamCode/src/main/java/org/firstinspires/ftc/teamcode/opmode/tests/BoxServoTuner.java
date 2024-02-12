package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Config
@TeleOp(name="Box Servo Tuner", group = "Tuner")
public class BoxServoTuner extends LinearOpMode {
    public static double inLock = 0.8;
    public static double outLock = 0.5;
    public static double inRel = 0.5;
    public static double outRel = 0.8;
    @Override
    public void runOpMode() throws InterruptedException {
        SimpleServo inner = new SimpleServo(hardwareMap, "innerServo", 0, 255);
        SimpleServo outer = new SimpleServo(hardwareMap, "outerServo", 0, 255);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            inner.setPosition(inLock);
            outer.setPosition(outLock);
        }
    }
}
