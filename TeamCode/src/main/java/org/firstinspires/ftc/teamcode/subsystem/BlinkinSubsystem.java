package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.*;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.BooleanSupplier;

public class BlinkinSubsystem extends SubsystemBase {
    private static boolean isStarted = false;
    private final ElapsedTime timer;
    private final RevBlinkinLedDriver blinkin;

    private RevBlinkinLedDriver.BlinkinPattern currentPattern;

    public BlinkinSubsystem(RevBlinkinLedDriver blinkin) {
        this.blinkin = blinkin;
        timer= new ElapsedTime();
    }



    public void setCurrentPattern(RevBlinkinLedDriver.BlinkinPattern currentPattern) {
        this.currentPattern = currentPattern;
    }

    public Command setPatternUntil(RevBlinkinLedDriver.BlinkinPattern currentPattern, BooleanSupplier until) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> setCurrentPattern(currentPattern)),
                new WaitUntilCommand(until),
                new InstantCommand(() -> setCurrentPattern(null))
        );
    }

    // time period is 120 seconds
    // last 30 seconds is end game
    @Override
    public void periodic() {
        if (!isStarted) {
                // we can start the timer here because the periodic function only runs after the start button is pressed. It's the same thing as referencing the state in the LinearOpMode
                timer.reset();
                isStarted = true;
        }
        if (currentPattern == null) {
            if (timer.seconds() > 110) {
                // set fast blinking for last 10 seconds
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_FAST);
            } else if (timer.seconds() > 90) {
                // set red powered LED for end game
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
            } else {
                // set rainbow powered LED for teleop
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
            }
        } else {
            blinkin.setPattern(currentPattern);
        }
    }
}
