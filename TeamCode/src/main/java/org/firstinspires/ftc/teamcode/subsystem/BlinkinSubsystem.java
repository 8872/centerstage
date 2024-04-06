package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BlinkinSubsystem extends SubsystemBase {
    private static boolean isStarted = false;
    static ElapsedTime timer;
    static RevBlinkinLedDriver blinkin;
    private static boolean external = false;

    public BlinkinSubsystem(RevBlinkinLedDriver blinkin) {
        BlinkinSubsystem.blinkin = blinkin;
    }

    public static double getElapsedTime() {
        return timer.seconds();
    }

    // this is to call in possibly during init stage where Command is not available
    public static void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        // check if it has not started otherwise invalid call - we only want this to be called during init stage
        // see Command below to use it with subsystems
        if(!isStarted) {
            blinkin.setPattern(pattern);
        }
    }

    public Command setColor(RevBlinkinLedDriver.BlinkinPattern color, boolean isFinished) {
        // if the pattern is hot pink, we don't want to change it and instead change back to our default pattern
        // who the fuck wants hot pink on their bot that's weird as hell. so it will be our extraneous case since idk how to override enum
        return new InstantCommand(() -> {
            if (isFinished) {
                external = false;
            } else {
                external = true;
                blinkin.setPattern(color);
            }
        });
    }
    // time period is 120 seconds
    // last 30 seconds is end game
    @Override
    public void periodic() {
        if (!isStarted) {
            // we can start the timer here because the periodic function only runs after the start button is pressed. It's the same thing as referencing the state in the LinearOpMode
            timer = new ElapsedTime();
            isStarted = true;
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
        }
        if (timer.seconds() > 110 && !external) {
            // set fast blinking for last 10 seconds
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_FAST);
        } else if (!external && timer.seconds() > 90) {
            // set red powered LED for end game
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        } else if (!external) {
            // set rainbow powered LED for teleop
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
        }
    }
}