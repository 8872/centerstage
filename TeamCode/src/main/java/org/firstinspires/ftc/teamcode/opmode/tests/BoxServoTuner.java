package org.firstinspires.ftc.teamcode.opmode.tests;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Box Servo Tuner", group = "Tuner")
@Disabled
public class BoxServoTuner extends LinearOpMode {
    public static double inLock = 0;
    public static double outLock = 0;
    public static double inRel = 0;
    public static double outRel = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        SimpleServo inner = new SimpleServo(hardwareMap, "in", 0, 255);
        SimpleServo outer = new SimpleServo(hardwareMap, "out", 0, 255);
        GamepadEx gamepadEx = new GamepadEx(gamepad1);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(new InstantCommand(()->inner.setPosition(inRel)));
            gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new InstantCommand(()->inner.setPosition(inLock)));
            gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(new InstantCommand(() -> outer.setPosition(outRel)));
            gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new InstantCommand(()-> outer.setPosition(outLock)));
        }
    }
}
