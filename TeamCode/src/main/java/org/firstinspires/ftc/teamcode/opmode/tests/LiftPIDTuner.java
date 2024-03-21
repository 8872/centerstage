package org.firstinspires.ftc.teamcode.opmode.tests;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.subsystem.LiftSys;

@TeleOp
public class LiftPIDTuner extends BaseOpMode {
    @Override
    public void initialize() {
        super.initialize();
        gb1(GamepadKeys.Button.A).whenPressed(liftSys.goTo(LiftSys.NONE));
        gb1(GamepadKeys.Button.B).whenPressed(liftSys.goTo(LiftSys.LOW));
        gb1(GamepadKeys.Button.X).whenPressed(liftSys.goTo(LiftSys.MID));
        gb1(GamepadKeys.Button.Y).whenPressed(liftSys.goTo(LiftSys.HIGH));
    }
}
