package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

@TeleOp(name = "Drive OpMode", group = "TeleOp")
public class DriveOpMode extends DriveBaseOpMode{


    @Override
    public void initialize() {
        super.initialize();
        gb1(GamepadKeys.Button.Y).whenPressed(liftSys.goTo(LiftSubsystem.THIRD));
        gb1(GamepadKeys.Button.B).whenPressed(liftSys.goTo(LiftSubsystem.SECOND));
        gb1(GamepadKeys.Button.X).whenPressed(liftSys.goTo(LiftSubsystem.FIRST));
        gb1(GamepadKeys.Button.A).whenPressed(liftSys.goTo(LiftSubsystem.NONE));
        gb1(GamepadKeys.Button.DPAD_LEFT).whenPressed(launcherSys.release());
        gb1(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new InstantCommand(()-> clawSys.cycleCommand()));
        gb1(GamepadKeys.Button.LEFT_BUMPER).toggleWhenPressed(armSys.home().andThen(clawSys.stopFirst(), clawSys.stopSecond()), armSys.away());


        //debugging
        gb2(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(clawSys.stopFirst());
        gb2(GamepadKeys.Button.LEFT_BUMPER).whenPressed(clawSys.stopSecond());
        gb2(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(clawSys.releaseFirst());
        gb2(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(clawSys.releaseSecond());
        register(drive, intakeSys, armSys, clawSys, liftSys, launcherSys);
        drive.setDefaultCommand(drive.drive(gamepadEx1::getLeftX,gamepadEx1::getLeftY,gamepadEx1::getRightX));
    }

}