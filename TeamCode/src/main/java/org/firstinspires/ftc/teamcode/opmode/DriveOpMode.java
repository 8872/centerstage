package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

@TeleOp(name = "Drive OpMode", group = "TeleOp")
public class DriveOpMode extends DriveBaseOpMode{

    @Override
    public void initialize() {
        super.initialize();

        gb1(GamepadKeys.Button.Y).whenPressed(new SequentialCommandGroup(new ParallelCommandGroup(boxSubsystem.stopFirst(),boxSubsystem.stopSecond()), new ParallelCommandGroup(armSubsystem.deposit(), liftSys.goTo(LiftSubsystem.THIRD))));
        gb1(GamepadKeys.Button.X).whenPressed(new SequentialCommandGroup(new ParallelCommandGroup(boxSubsystem.stopFirst(),boxSubsystem.stopSecond()), new ParallelCommandGroup(armSubsystem.deposit(), liftSys.goTo(LiftSubsystem.SECOND))));
        gb1(GamepadKeys.Button.B).whenPressed(new SequentialCommandGroup(new ParallelCommandGroup(boxSubsystem.stopFirst(),boxSubsystem.stopSecond()), new ParallelCommandGroup(armSubsystem.deposit(), liftSys.goTo(LiftSubsystem.FIRST))));
        gb1(GamepadKeys.Button.A).whenPressed(new SequentialCommandGroup(armSubsystem.intake(), liftSys.goTo(LiftSubsystem.NONE)));
//        gb1(GamepadKeys.Button.DPAD_LEFT).whenPressed(launcherSubsystem.release());
        gb1(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new InstantCommand(()-> boxSubsystem.depositNext()));
        gb1(GamepadKeys.Button.DPAD_DOWN).toggleWhenPressed(armSubsystem.intake().andThen(boxSubsystem.stopFirst(), boxSubsystem.stopSecond()), armSubsystem.deposit());
        gb1(GamepadKeys.Button.LEFT_BUMPER).whileHeld(drive.drive(gamepadEx1::getLeftX,gamepadEx1::getLeftY,gamepadEx1::getRightX));
        gb1(GamepadKeys.Button.DPAD_UP).whenPressed(new ParallelCommandGroup(armSubsystem.deposit(), liftSys.goTo(-700)));
        //debugging
        gb2(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(boxSubsystem.stopFirst());
        gb2(GamepadKeys.Button.LEFT_BUMPER).whenPressed(boxSubsystem.stopSecond());
        gb2(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(boxSubsystem.releaseFirst());
        gb2(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(boxSubsystem.releaseSecond());
        gb2(GamepadKeys.Button.DPAD_UP).whenPressed(intakeSys.setHeight(IntakeSubsystem.HIGH));
        gb2(GamepadKeys.Button.DPAD_DOWN).whenPressed(intakeSys.setHeight(IntakeSubsystem.LOW));
        register(drive, intakeSys, armSubsystem, boxSubsystem, liftSys, launcherSubsystem);
        drive.setDefaultCommand(drive.drive(gamepadEx1::getLeftX,gamepadEx1::getLeftY,gamepadEx1::getRightX));
    }

}