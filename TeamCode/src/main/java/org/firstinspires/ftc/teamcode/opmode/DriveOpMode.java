package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

@TeleOp(name = "Drive OpMode", group = "TeleOp")
public class DriveOpMode extends DriveBaseOpMode{

    @Override
    public void initialize() {
        super.initialize();
        //gb1(GamepadKeys.Button.A).whenHeld(new ParallelCommandGroup(intakeSys.in(),intakeSys.setHeight(IntakeSubsystem.LOW))).whenReleased(new ParallelCommandGroup(intakeSys.stop(), intakeSys.setHeight(IntakeSubsystem.HIGH)));
        //gb1(GamepadKeys.Button.LEFT_STICK_BUTTON).and(gb1(GamepadKeys.Button.A)).whenActive(new ParallelCommandGroup(intakeSys.out(),intakeSys.setHeight(IntakeSubsystem.LOW))).whenInactive(new ParallelCommandGroup(intakeSys.stop(), intakeSys.setHeight(IntakeSubsystem.HIGH)));
        gb1(GamepadKeys.Button.Y).whenPressed(liftSys.goTo(LiftSubsystem.THIRD));
        gb1(GamepadKeys.Button.B).whenPressed(liftSys.goTo(LiftSubsystem.SECOND));
        gb1(GamepadKeys.Button.X).whenPressed(liftSys.goTo(LiftSubsystem.FIRST));
        gb1(GamepadKeys.Button.A).whenPressed(liftSys.goTo(LiftSubsystem.NONE));

        gb1(GamepadKeys.Button.DPAD_LEFT).whenPressed(launcherSys.release());
        gb1(GamepadKeys.Button.RIGHT_BUMPER).toggleWhenPressed(armSys.home(), armSys.away());
        gb1(GamepadKeys.Button.LEFT_BUMPER).whileHeld(drive.drive(gamepadEx1::getLeftX,gamepadEx1::getLeftY,gamepadEx1::getRightX,0.5));//debugging
        gb2(GamepadKeys.Button.A).whenPressed(armSys.pitchHome());
        gb2(GamepadKeys.Button.X).whenPressed(armSys.pitchAway());
        gb2(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new InstantCommand(()-> armServo.setPosition(axonServoAway)));
        gb2(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new InstantCommand(()-> armServo.setPosition(axonServoHome)));
        gb2(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(new InstantCommand(()-> pitchServo.setPosition(pitchServoHome)));
        gb2(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(new InstantCommand(()-> pitchServo.setPosition(pitchServoAway)));
        register(drive, intakeSys, armSys, clawSys, liftSys, launcherSys);
        drive.setDefaultCommand(drive.drive(gamepadEx1::getLeftX,gamepadEx1::getLeftY,gamepadEx1::getRightX));
    }
}