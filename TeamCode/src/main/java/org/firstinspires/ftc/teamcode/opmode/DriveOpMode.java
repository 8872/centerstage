package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.command.drive.DriveRobotCentric;
import org.firstinspires.ftc.teamcode.command.intake.SetIntake;
import org.firstinspires.ftc.teamcode.command.intake.SetIntakeDirection;
import org.firstinspires.ftc.teamcode.command.lift.SetHeight;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSys;
import org.firstinspires.ftc.teamcode.subsystem.LauncherSys;
import org.firstinspires.ftc.teamcode.subsystem.LiftSys;

@TeleOp(name = "Drive OpMode", group = "TeleOp")
public class DriveOpMode extends DriveBaseOpMode{
    @Override
    public void initialize() {
        super.initialize();
        DriveRobotCentric robotCentricDrive = new DriveRobotCentric(drive, gamepadEx1::getLeftX,
                gamepadEx1::getLeftY, gamepadEx1::getRightX);
        SetIntake startIntake = new SetIntake(intakeSys, IntakeSys.IntakeDirection.IN, IntakeSys.StackHeight.LOW);
        SetIntake stopIntake = new SetIntake(intakeSys, IntakeSys.IntakeDirection.STOP, IntakeSys.StackHeight.HIGH);
        SetIntake backIntake = new SetIntake(intakeSys, IntakeSys.IntakeDirection.OUT, IntakeSys.StackHeight.LOW);
        SetHeight none = new SetHeight(liftSys, LiftSys.Height.NONE);
        SetHeight low = new SetHeight(liftSys, LiftSys.Height.LOW);
        SetHeight medium = new SetHeight(liftSys, LiftSys.Height.MEDIUM);
        SetHeight high = new SetHeight(liftSys, LiftSys.Height.HIGH);
        gb1(GamepadKeys.Button.A).toggleWhenPressed(startIntake, stopIntake);
        gb1(GamepadKeys.Button.B).toggleWhenPressed(backIntake, stopIntake);
        gb1(GamepadKeys.Button.DPAD_UP).whenPressed(high);
        gb1(GamepadKeys.Button.DPAD_DOWN).whenPressed(none);
        gb1(GamepadKeys.Button.DPAD_LEFT).whenPressed(low);
        gb1(GamepadKeys.Button.DPAD_RIGHT).whenPressed(medium);
        gb1(GamepadKeys.Button.START).whenPressed(launcherSys.release());
        gb1(GamepadKeys.Button.RIGHT_BUMPER).toggleWhenPressed(armSys.home(), armSys.home());

        //debugging
        gb2(GamepadKeys.Button.A).whenPressed(armSys.pitchHome());
        gb2(GamepadKeys.Button.X).whenPressed(armSys.pitchAway());
        gb2(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new InstantCommand(()-> armServo.setPosition(axonServoAway)));
        gb2(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new InstantCommand(()-> armServo.setPosition(axonServoHome)));
        gb2(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(new InstantCommand(()-> pitchServo.setPosition(pitchServoHome)));
        gb2(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(new InstantCommand(()-> pitchServo.setPosition(pitchServoAway)));
        register(drive, intakeSys, armSys, clawSys, liftSys, launcherSys);
        drive.setDefaultCommand(robotCentricDrive);
    }
}