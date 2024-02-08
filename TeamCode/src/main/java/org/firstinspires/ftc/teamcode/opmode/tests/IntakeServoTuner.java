package org.firstinspires.ftc.teamcode.opmode.tests;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.apache.commons.math3.util.Precision;

@TeleOp(name="Intake Servo Tuner", group = "Tuner")
@Disabled
public class IntakeServoTuner extends LinearOpMode {
    public static double intakeServoHigh = 0;
    public static double intakeServoLow = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        SimpleServo intakeServo = new SimpleServo(hardwareMap, "intakeServo", 0, 255);
        GamepadEx gamepadEx = new GamepadEx(gamepad1);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            gamepadEx.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(()->intakeServo.setPosition(intakeServoHigh)));
            gamepadEx.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(()-> intakeServo.setPosition(intakeServoLow)));
        }
    }
}
