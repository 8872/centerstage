package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Main Op Mode")
public class MainOpMode extends BaseOpMode {
    @Override
    public void initialize() {
        super.initialize();
        gb1(GamepadKeys.Button.LEFT_BUMPER).whileHeld(driveSys.drive(gamepadEx1::getRightX,gamepadEx1::getLeftY,gamepadEx1::getLeftX, 0.5));
        gb2(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(boxSys.release());
        gb2(GamepadKeys.Button.LEFT_BUMPER).toggleWhenPressed(new ParallelCommandGroup(armSys.deposit(), boxSys.close()), new ParallelCommandGroup(armSys.intake(), boxSys.intake()));
        register(boxSys, armSys, driveSys, intakeSys);
        intakeSys.setDefaultCommand(intakeSys.intake(()->gamepadEx1.gamepad.right_trigger, ()->gamepadEx1.gamepad.left_trigger));
        driveSys.setDefaultCommand(driveSys.drive(gamepadEx1::getRightX,gamepadEx1::getLeftY,gamepadEx1::getLeftX, 1));
    }
}
