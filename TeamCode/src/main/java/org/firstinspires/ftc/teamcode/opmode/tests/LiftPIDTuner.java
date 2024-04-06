package org.firstinspires.ftc.teamcode.opmode.tests;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

@TeleOp
public class LiftPIDTuner extends BaseOpMode {
    @Override
    public void initialize() {
        super.initialize();
        gb1(GamepadKeys.Button.A).whenPressed(liftSubsystem.goTo(LiftSubsystem.NONE));
        gb1(GamepadKeys.Button.B).whenPressed(liftSubsystem.goTo(LiftSubsystem.LOW));
        gb1(GamepadKeys.Button.X).whenPressed(liftSubsystem.goTo(LiftSubsystem.MID));
        gb1(GamepadKeys.Button.Y).whenPressed(liftSubsystem.goTo(LiftSubsystem.HIGH));
    }
}
