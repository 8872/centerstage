package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

@TeleOp(name = "CoopOpMode", group = "TeleOp")
public class CoopOpMode extends DriveBaseOpMode {

    @Override
    public void initialize() {
        super.initialize();

        gb2(GamepadKeys.Button.Y).whenPressed(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(boxSubsystem.stopFirst(), boxSubsystem.stopSecond()),
                        new ParallelCommandGroup(armSubsystem.deposit(), liftSys.goTo(LiftSubsystem.THIRD))
                )
        );
        gb2(GamepadKeys.Button.X).whenPressed(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(boxSubsystem.stopFirst(), boxSubsystem.stopSecond()),
                        new ParallelCommandGroup(armSubsystem.deposit(), liftSys.goTo(LiftSubsystem.SECOND))
                )
        );
        gb2(GamepadKeys.Button.B).whenPressed(
                new SequentialCommandGroup(new ParallelCommandGroup(boxSubsystem.stopFirst(), boxSubsystem.stopSecond()),
                        new ParallelCommandGroup(armSubsystem.deposit(), liftSys.goTo(LiftSubsystem.FIRST))
                )
        );
        gb2(GamepadKeys.Button.A).whenPressed(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(boxSubsystem.releaseFirst(), boxSubsystem.releaseSecond(), armSubsystem.intake()),
                        liftSys.goTo(LiftSubsystem.NONE)
                )
        );
        gb2(GamepadKeys.Button.DPAD_RIGHT).whenPressed(launcherSubsystem.release());
        gb2(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(boxSubsystem.depositNext());
        gb2(GamepadKeys.Button.DPAD_DOWN).whenPressed(new ParallelCommandGroup(boxSubsystem.stopFirst(), boxSubsystem.stopSecond()));
        gb2(GamepadKeys.Button.DPAD_UP).whenPressed(new ParallelCommandGroup(boxSubsystem.releaseFirst(), boxSubsystem.releaseSecond()));
        gb2(GamepadKeys.Button.DPAD_LEFT).toggleWhenPressed(
                armSubsystem.intake().andThen(boxSubsystem.releaseFirst(), boxSubsystem.releaseSecond()),
                armSubsystem.deposit().andThen(boxSubsystem.stopSecond(), boxSubsystem.stopFirst())
        );

        gb1(GamepadKeys.Button.LEFT_BUMPER).whenHeld(drive.drive(gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX, 0.5));


        register(drive, intakeSys, armSubsystem, boxSubsystem, liftSys, launcherSubsystem);
        launcherSubsystem.setDefaultCommand(launcherSubsystem.move(gamepadEx2::getLeftY));
        liftSys.setDefaultCommand(liftSys.manualSetHeight(gamepadEx2::getRightY));
        intakeSys.setDefaultCommand(intakeSys.intake(gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));
        drive.setDefaultCommand(drive.drive(gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX));
    }

}