package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystem.LiftSys;
import org.firstinspires.ftc.teamcode.util.commands.DelayedCommand;

import java.util.concurrent.Delayed;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;

@Config
@TeleOp
public class MainOpMode extends BaseOpMode {

    public static int delay = 250;

    @Override
    public void initialize() {
        super.initialize();

        register(boxSys, armSys, driveSys, intakeSys, localizerSys, liftSys);

        // plane
//        new GamepadButton(gamepadEx2, LEFT_STICK_BUTTON)
//                .and(new GamepadButton(gamepadEx2, RIGHT_STICK_BUTTON))
//                .and(new GamepadButton(gamepadEx2, A)).whenActive(planeSys.launch());
        gb2(DPAD_RIGHT).whenPressed(planeSys.launch());

        // arm + box
        gb2(RIGHT_BUMPER).whenPressed(new InstantCommand(() -> boxSys.release()));
        gb2(LEFT_BUMPER).toggleWhenPressed(
                new ParallelCommandGroup(armSys.intake(), boxSys.intake()),
                new ParallelCommandGroup(armSys.deposit(), boxSys.close())
        );

        // slide + box
        gb2(A).whenPressed(new ParallelCommandGroup(
                liftSys.goTo(LiftSys.NONE), new ParallelCommandGroup(armSys.intake(), boxSys.intake())
        ));
        slideUp(B, LiftSys.LOW);
        slideUp(X, LiftSys.MID);
        slideUp(Y, LiftSys.HIGH);


//        gb2(DPAD_LEFT).toggleWhenPressed(
//                new ParallelCommandGroup(armSys.intake(), boxSys.intake()),
//                new ParallelCommandGroup(armSys.deposit(), boxSys.close())
//        );

        gb1(LEFT_BUMPER).whileHeld(
                driveSys.slow(gamepadEx1::getRightX, gamepadEx1::getLeftY, gamepadEx1::getLeftX));



//        liftSys.setDefaultCommand(liftSys.manualSetHeight(gamepadEx2::getRightY));
        intakeSys.setDefaultCommand(intakeSys.intake(() -> gamepadEx2.gamepad.right_trigger, () -> gamepadEx2.gamepad.left_trigger));
        driveSys.setDefaultCommand(driveSys.drive(gamepadEx1::getRightX, gamepadEx1::getLeftY, gamepadEx1::getLeftX));



    }

    private void slideUp(GamepadKeys.Button button, double height) {
        gb2(button).whenPressed(new ParallelCommandGroup(
                liftSys.goTo(height), new ParallelCommandGroup(armSys.deposit(), boxSys.close())
        ));
    }


    private void slideUpDelayed(GamepadKeys.Button button, double height) {
        gb2(button).whenPressed(new ParallelCommandGroup(
                liftSys.goTo(height), new DelayedCommand(new ParallelCommandGroup(armSys.deposit(), boxSys.close()), delay)
        ));
    }
}
