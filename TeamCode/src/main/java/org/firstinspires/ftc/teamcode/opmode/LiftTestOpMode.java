package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

@TeleOp(name = "Lift OpMode", group = "TeleOp")
public class LiftTestOpMode extends DriveBaseOpMode{

    @Override
    public void initialize() {
        super.initialize();
        gb1(GamepadKeys.Button.Y).whenPressed(liftSys.goTo(LiftSubsystem.THIRD));
        gb1(GamepadKeys.Button.B).whenPressed(liftSys.goTo(LiftSubsystem.SECOND));
        gb1(GamepadKeys.Button.X).whenPressed(liftSys.goTo(LiftSubsystem.FIRST));
        gb1(GamepadKeys.Button.A).whenPressed(liftSys.goTo(LiftSubsystem.NONE));
        register(drive, intakeSys, armSubsystem, boxSubsystem, liftSys, launcherSubsystem);
        drive.setDefaultCommand(drive.drive(gamepadEx1::getLeftX,gamepadEx1::getLeftY,gamepadEx1::getRightX));
    }

}