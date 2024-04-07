package org.firstinspires.ftc.teamcode.opmode;

import android.util.Log;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.*;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.util.commands.DelayedCommand;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;

@Config
@TeleOp
public class MainOpMode extends BaseOpMode {

    @Override
    public void initialize() {
        super.initialize();

        register(boxSubsystem, armSubsystem, driveSubsystem, intakeSubsystem, localizerSubsystem, liftSubsystem);

        // plane
//        new GamepadButton(gamepadEx2, LEFT_STICK_BUTTON)
//                .and(new GamepadButton(gamepadEx2, RIGHT_STICK_BUTTON))
//                .and(new GamepadButton(gamepadEx2, A)).whenActive(planeSys.launch());
        gb2(DPAD_RIGHT).whenPressed(planeSubsystem.launch());

        // arm + box
        gb2(RIGHT_BUMPER).whenPressed(new InstantCommand(() -> boxSubsystem.release()));
        gb2(LEFT_BUMPER).toggleWhenPressed(
                new ParallelCommandGroup(armSubsystem.intake(), boxSubsystem.intake()),
                new ParallelCommandGroup(armSubsystem.deposit(), boxSubsystem.close())
        );

        // slide + box
        gb2(A).whenPressed(new ParallelCommandGroup(
                liftSubsystem.goTo(LiftSubsystem.NONE), new ParallelCommandGroup(armSubsystem.intake(), boxSubsystem.intake())
        ));
        slideUp(B, LiftSubsystem.LOW);
        slideUp(X, LiftSubsystem.MID);
        slideUp(Y, LiftSubsystem.HIGH);

        gb2(DPAD_UP).whenPressed(hangSubsystem.delayed(1, 3000));
        gb2(DPAD_DOWN).whenPressed(hangSubsystem.delayed(-1, 3000));

        gb2(DPAD_LEFT).toggleWhenPressed(
                new ParallelCommandGroup(armSubsystem.intake(), boxSubsystem.intake()),
                new ParallelCommandGroup(armSubsystem.deposit(), boxSubsystem.close())
        );

        gb1(LEFT_BUMPER).whileHeld(driveSubsystem.slow(gamepadEx1::getRightX, gamepadEx1::getLeftY, gamepadEx1::getLeftX));
//                .alongWith(blinkinSubsystem.setPatternUntil(RevBlinkinLedDriver.BlinkinPattern.YELLOW, () -> !gb1(LEFT_BUMPER).get())));


//        liftSys.setDefaultCommand(liftSys.manualSetHeight(gamepadEx2::getRightY));
        intakeSubsystem.setDefaultCommand(intakeSubsystem.intake(() -> gamepadEx2.gamepad.right_trigger, () -> gamepadEx2.gamepad.left_trigger));
        driveSubsystem.setDefaultCommand(driveSubsystem.drive(gamepadEx1::getRightX, gamepadEx1::getLeftY, gamepadEx1::getLeftX));


    }

    private void slideUp(GamepadKeys.Button button, double height) {
        gb2(button).whenPressed(new ParallelCommandGroup(
                liftSubsystem.goTo(height), new ParallelCommandGroup(armSubsystem.deposit(), boxSubsystem.close())
        ));
    }


//    private void slideUpDelayed(GamepadKeys.Button button, double height) {
//        gb2(button).whenPressed(new ParallelCommandGroup(
//                liftSys.goTo(height), new DelayedCommand(new ParallelCommandGroup(armSys.deposit(), boxSys.close()), delay)
//        ));
//    }
}
