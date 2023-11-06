package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.command.lift.SetHeight;
import org.firstinspires.ftc.teamcode.subsystem.LiftSys;

@TeleOp(name = "Drive OpMode", group = "TeleOp")
public class DriveOpMode extends DriveBaseOpMode{

    @Override
    public void initialize() {
        super.initialize();
        SetHeight none = new SetHeight(liftSys, LiftSys.Height.NONE);
        SetHeight low = new SetHeight(liftSys, LiftSys.Height.LOW);
        SetHeight medium = new SetHeight(liftSys, LiftSys.Height.MEDIUM);
        SetHeight high = new SetHeight(liftSys, LiftSys.Height.HIGH);
        //gb1(GamepadKeys.Button.A).whenHeld(new ParallelCommandGroup(intakeSys.in(),intakeSys.setHeight(IntakeSubsystem.LOW))).whenReleased(new ParallelCommandGroup(intakeSys.stop(), intakeSys.setHeight(IntakeSubsystem.HIGH)));
        //gb1(GamepadKeys.Button.LEFT_STICK_BUTTON).and(gb1(GamepadKeys.Button.A)).whenActive(new ParallelCommandGroup(intakeSys.out(),intakeSys.setHeight(IntakeSubsystem.LOW))).whenInactive(new ParallelCommandGroup(intakeSys.stop(), intakeSys.setHeight(IntakeSubsystem.HIGH)));
        gb1(GamepadKeys.Button.Y).whenPressed(high);
        gb1(GamepadKeys.Button.A).whenPressed(none);
        gb1(GamepadKeys.Button.B).whenPressed(low);
        gb1(GamepadKeys.Button.X).whenPressed(medium);
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