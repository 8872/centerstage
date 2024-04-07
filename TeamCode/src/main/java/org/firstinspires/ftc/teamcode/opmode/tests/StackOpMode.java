package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.HangSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;

@Config
@TeleOp(name = "Test Stacks OpMode")
public class StackOpMode extends BaseOpMode {
    public static double slow = 0.35;
    @Override
    public void initialize() {
        super.initialize();

        register(boxSubsystem, armSubsystem, driveSubsystem, intakeSubsystem, localizerSubsystem, liftSubsystem);

        // plane
//        new GamepadButton(gamepadEx2, LEFT_STICK_BUTTON)
//                .and(new GamepadButton(gamepadEx2, RIGHT_STICK_BUTTON))
//                .and(new GamepadButton(gamepadEx2, A)).whenActive(planeSys.launch());
        gb1(DPAD_RIGHT).whenPressed(planeSubsystem.launch());

        // arm + box
        gb2(RIGHT_BUMPER).whenPressed(new InstantCommand(() -> boxSubsystem.release()));
        gb2(LEFT_BUMPER).toggleWhenPressed(
                new ParallelCommandGroup(armSubsystem.intake(), boxSubsystem.intake()),
                new ParallelCommandGroup(armSubsystem.deposit(), boxSubsystem.close())
        );

        // slide + box
        gb1(A).whenPressed(new ParallelCommandGroup(
                liftSubsystem.goTo(LiftSubsystem.NONE), new ParallelCommandGroup(armSubsystem.intake(), boxSubsystem.intake())
        ));
        slideUp(B, LiftSubsystem.LOW);
        slideUp(X, LiftSubsystem.MID);
        slideUp(Y, LiftSubsystem.HIGH);

        gb1(DPAD_UP).whenPressed(hangSubsystem.delayed(1, 3000));
        gb1(DPAD_DOWN).whenPressed(hangSubsystem.delayed(-1, 3000));

        gb1(DPAD_LEFT).toggleWhenPressed(
                new ParallelCommandGroup(armSubsystem.intake(), boxSubsystem.intake()),
                new ParallelCommandGroup(armSubsystem.deposit(), boxSubsystem.close())
        );

        gb1(LEFT_BUMPER).whileHeld(driveSubsystem.drive(gamepadEx1::getRightX, gamepadEx1::getLeftY, gamepadEx1::getLeftX, 0.1));
//                .alongWith(blinkinSubsystem.setPatternUntil(RevBlinkinLedDriver.BlinkinPattern.YELLOW, () -> !gb1(LEFT_BUMPER).get())));

//        gb1(RIGHT_BUMPER).whileHeld(intakeSubsystem.slow(() -> gamepadEx1.gamepad.right_trigger, () -> gamepadEx1.gamepad.left_trigger));
        gb1(RIGHT_BUMPER).whileHeld(() -> IntakeSubsystem.intakeInPower = slow).whenReleased(() -> IntakeSubsystem.intakeInPower = 0.67);
//        liftSys.setDefaultCommand(liftSys.manualSetHeight(gamepadEx2::getRightY));
        intakeSubsystem.setDefaultCommand(intakeSubsystem.intake(() -> gamepadEx1.gamepad.right_trigger, () -> gamepadEx1.gamepad.left_trigger));
        driveSubsystem.setDefaultCommand(driveSubsystem.drive(gamepadEx1::getRightX, gamepadEx1::getLeftY, gamepadEx1::getLeftX));


    }

    private void slideUp(GamepadKeys.Button button, double height) {
        gb1(button).whenPressed(new ParallelCommandGroup(
                liftSubsystem.goTo(height), new ParallelCommandGroup(armSubsystem.deposit(), boxSubsystem.close())
        ));
    }


//    private void slideUpDelayed(GamepadKeys.Button button, double height) {
//        gb2(button).whenPressed(new ParallelCommandGroup(
//                liftSys.goTo(height), new DelayedCommand(new ParallelCommandGroup(armSys.deposit(), boxSys.close()), delay)
//        ));
//    }
}
