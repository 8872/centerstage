package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.message.redux.ReceiveGamepadState;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystem.LiftSys;

@Config
@TeleOp(name = "Main Op Mode")
public class MainOpMode extends BaseOpMode {
    @Override
    public void initialize() {
        super.initialize();
        new GamepadButton(gamepadEx2, GamepadKeys.Button.LEFT_STICK_BUTTON)
                .and(new GamepadButton(gamepadEx2, GamepadKeys.Button.RIGHT_STICK_BUTTON))
                .and(new GamepadButton(gamepadEx2, GamepadKeys.Button.A))
                .whenActive(planeSys.launch());
        //gb1(GamepadKeys.Button.A).whenPressed(liftSys.goTo(400));
        //gb1(GamepadKeys.Button.LEFT_BUMPER).whileHeld(driveSys.drive(gamepadEx1::getRightX,gamepadEx1::getLeftY,gamepadEx1::getLeftX, 0.5));
        gb2(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(boxSys.release());
        gb1(GamepadKeys.Button.LEFT_BUMPER).toggleWhenPressed(new ParallelCommandGroup(armSys.deposit()), new ParallelCommandGroup(armSys.intake()));
        register(boxSys, armSys, driveSys, intakeSys, localizerSys);
        intakeSys.setDefaultCommand(intakeSys.intake(()->gamepadEx1.gamepad.right_trigger, ()->gamepadEx1.gamepad.left_trigger));
        driveSys.setDefaultCommand(driveSys.drive(gamepadEx1::getRightX,gamepadEx1::getLeftY,gamepadEx1::getLeftX, 1));
        gb2(GamepadKeys.Button.Y).whenPressed(new InstantCommand(() -> liftSys.setHeight(LiftSys.HIGH)));
        gb2(GamepadKeys.Button.X).whenPressed(new InstantCommand(() -> liftSys.setHeight(LiftSys.MID)));
        gb2(GamepadKeys.Button.B).whenPressed(new InstantCommand(() -> liftSys.setHeight(LiftSys.LOW)));
        gb2(GamepadKeys.Button.A).whenPressed(new InstantCommand(() -> liftSys.setHeight(LiftSys.NONE)));
        gb2(GamepadKeys.Button.DPAD_LEFT).toggleWhenPressed(armSys.intake(), armSys.deposit());

    }
}
