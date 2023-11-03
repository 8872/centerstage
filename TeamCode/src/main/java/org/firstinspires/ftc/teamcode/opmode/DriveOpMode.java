package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.command.drive.DriveRobotCentric;
import org.firstinspires.ftc.teamcode.command.intake.SetIntake;
import org.firstinspires.ftc.teamcode.command.intake.SetIntakeDirection;
import org.firstinspires.ftc.teamcode.command.lift.SetHeight;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSys;
import org.firstinspires.ftc.teamcode.subsystem.LiftSys;

@TeleOp(name = "Drive OpMode", group = "TeleOp")
public class DriveOpMode extends DriveBaseOpMode{
    private DriveRobotCentric robotCentricDrive;
    private SetIntakeDirection startIntake, stopIntake, backIntake;
    private SetHeight none, low, medium, high;
    @Override
    public void initialize() {
        super.initialize();
        robotCentricDrive = new DriveRobotCentric(drive, gamepadEx1::getLeftX,
                gamepadEx1::getLeftY, gamepadEx1::getRightX);
        startIntake = new SetIntakeDirection(intakeSys, IntakeSys.IntakeDirection.IN);
        stopIntake = new SetIntakeDirection(intakeSys, IntakeSys.IntakeDirection.STOP);
        backIntake = new SetIntakeDirection(intakeSys, IntakeSys.IntakeDirection.OUT);
        none = new SetHeight(liftSys, LiftSys.Height.NONE);
        low = new SetHeight(liftSys, LiftSys.Height.LOW);
        medium = new SetHeight(liftSys, LiftSys.Height.MEDIUM);
        high = new SetHeight(liftSys, LiftSys.Height.HIGH);
        gb1(GamepadKeys.Button.A).toggleWhenPressed(startIntake, stopIntake);
        gb1(GamepadKeys.Button.B).toggleWhenPressed(backIntake, stopIntake);
        gb1(GamepadKeys.Button.DPAD_UP).whenPressed(high);
        gb1(GamepadKeys.Button.DPAD_DOWN).whenPressed(none);
        gb1(GamepadKeys.Button.DPAD_LEFT).whenPressed(low);
        gb1(GamepadKeys.Button.DPAD_RIGHT).whenPressed(medium);
        register(drive, intakeSys);
        drive.setDefaultCommand(robotCentricDrive);
    }
}