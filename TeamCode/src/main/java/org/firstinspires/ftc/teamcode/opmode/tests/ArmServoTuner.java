package org.firstinspires.ftc.teamcode.opmode.tests;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

@TeleOp(name="Arm Servo Tuner", group = "Tuner")
@Disabled
public class ArmServoTuner extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        SimpleServo arm = new SimpleServo(hardwareMap, "armServo", 0, 355);
        SimpleServo pitch = new SimpleServo(hardwareMap, "pitchServo", 0, 355);
        GamepadEx gamepadEx = new GamepadEx(gamepad1);
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            gamepadEx.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(()->arm.setPosition(ArmSubsystem.armIntake)));
            gamepadEx.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(()-> arm.setPosition(ArmSubsystem.armOuttake)));
            gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new InstantCommand(()-> pitch.setPosition(ArmSubsystem.pitchOuttake)));
            gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new InstantCommand(()-> pitch.setPosition(ArmSubsystem.pitchIntake)));
        }
    }
}
