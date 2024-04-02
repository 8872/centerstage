package org.firstinspires.ftc.teamcode.opmode.tests;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.apache.commons.math3.util.Precision;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSys;

@TeleOp(name="Intake Servo Tuner", group = "Tuner")
public class IntakeServoTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SimpleServo stackRight = new SimpleServo(hardwareMap, "stack", 0, 255);
        SimpleServo stackLeft = new SimpleServo(hardwareMap, "stack2", 0, 255);
        GamepadEx gamepadEx = new GamepadEx(gamepad1);

        MotorEx intake = new MotorEx(hardwareMap, "intake", Motor.GoBILDA.RPM_1150);

        stackRight.setPosition(IntakeSys.intakeServoHighPosition);
        stackLeft.setPosition(IntakeSys.intakeServoHighPosition);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            if(gamepad1.x){
                stackRight.setPosition(IntakeSys.intakeServoLowPosition);
            }if(gamepad1.y){
                stackRight.setPosition(IntakeSys.intakeServoHighPosition);
            }if(gamepad1.dpad_left){
                stackLeft.setPosition(IntakeSys.intakeServoLowPosition);
            }if (gamepad1.dpad_up) {
                stackLeft.setPosition(IntakeSys.intakeServoHighPosition);
            }

            if(Math.abs(gamepad1.left_stick_y)>0.01){
                stackLeft.setPosition(stackLeft.getPosition() + (gamepad1.left_stick_y*0.005));
            }
            if(Math.abs(gamepad1.right_stick_y)>0.01){
                stackRight.setPosition(stackRight.getPosition() + (gamepad1.right_stick_y*0.005));
            }

            if(gamepad1.right_trigger>0.1){
                intake.set(0.7);
            }else if(gamepad1.left_trigger>0.1){
                intake.set(-0.7);
            }else{
                intake.set(0);
            }

            telemetry.addData("Right Servo Pos:", stackRight.getPosition());
            telemetry.addData("Left Servo Pos:", stackLeft.getPosition());
//            gamepadEx.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(()->stackRight.setPosition(IntakeSys.intakeServoLowPosition)));
//            gamepadEx.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(()-> stackRight.setPosition(IntakeSys.intakeServoHighPosition)));
//            gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(()->stackLeft.setPosition(IntakeSys.intakeServoLowPosition)));
//            gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(()-> stackLeft.setPosition(IntakeSys.intakeServoHighPosition)));
        }
    }
}
