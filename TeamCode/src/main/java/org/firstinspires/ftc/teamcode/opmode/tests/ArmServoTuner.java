package org.firstinspires.ftc.teamcode.opmode.tests;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Arm Servo Tuner", group = "Tuner")

public class ArmServoTuner extends LinearOpMode {

    public static double armHome;
    public static double armAway;
    public static double pitchHome;
    public static double pitchAway;
    @Override
    public void runOpMode() throws InterruptedException {
        SimpleServo arm = new SimpleServo(hardwareMap, "arm", 0, 355);
        SimpleServo pitch = new SimpleServo(hardwareMap, "pitch", 0, 355);
        GamepadEx gamepadEx = new GamepadEx(gamepad1);
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            gamepadEx.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(()->arm.setPosition(armHome)));
            gamepadEx.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(()-> arm.setPosition(armAway)));
            gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new InstantCommand(()-> pitch.setPosition(pitchAway)));
            gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new InstantCommand(()-> pitch.setPosition(pitchHome)));
        }
    }
}
