package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
@Config
@TeleOp(name="Intake Servo Tuner", group = "Tuner")
public class IntakeServoTuner extends LinearOpMode {
    public static boolean invert1 = false;
    public static boolean invert2 = true;
    @Override
    public void runOpMode() throws InterruptedException {
        SimpleServo stackRight = new SimpleServo(hardwareMap, "stack", 0, 255);
        SimpleServo stackLeft = new SimpleServo(hardwareMap, "stack2", 0, 255);
        if (invert1) stackLeft.setInverted(true);
        if (invert2) stackRight.setInverted(true);
        GamepadEx gamepadEx = new GamepadEx(gamepad1);

        MotorEx intake = new MotorEx(hardwareMap, "intake", Motor.GoBILDA.RPM_1150);

        stackRight.setPosition(IntakeSubsystem.servoHighPosition);
        stackLeft.setPosition(IntakeSubsystem.servoHighPosition);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            if(gamepad1.x){
                stackRight.setPosition(IntakeSubsystem.servoLowPosition);
            }if(gamepad1.y){
                stackRight.setPosition(IntakeSubsystem.servoHighPosition);
            }if(gamepad1.dpad_left){
                stackLeft.setPosition(IntakeSubsystem.servo2LowPosition);
            }if (gamepad1.dpad_up) {
                stackLeft.setPosition(IntakeSubsystem.servo2HighPosition);
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
            telemetry.update();
//            gamepadEx.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(()->stackRight.setPosition(IntakeSys.intakeServoLowPosition)));
//            gamepadEx.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(()-> stackRight.setPosition(IntakeSys.intakeServoHighPosition)));
//            gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(()->stackLeft.setPosition(IntakeSys.intakeServoLowPosition)));
//            gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(()-> stackLeft.setPosition(IntakeSys.intakeServoHighPosition)));
        }
    }
}
