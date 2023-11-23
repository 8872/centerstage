package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;

@Config
public class LauncherSubsystem extends SubsystemBase {
    SimpleServo launcherHeightServo;
    SimpleServo launcherServo;

    public static double launcherPos = 0.5;
    public static double launcherHeightPos = 0.5;
    public static double IDLE = 0.7;

    private boolean launchStage = false;
    public LauncherSubsystem(SimpleServo launcherHeightServo, SimpleServo launcherServo) {
        this.launcherHeightServo = launcherHeightServo;
        this.launcherServo = launcherServo;

    }

    public Command release() {
        if (!launchStage) {
            launchStage = true;
            return new InstantCommand(() -> launcherHeightServo.setPosition(launcherHeightPos));
        } else {
            launcherHeightServo.disable();
            return new InstantCommand(() -> launcherServo.setPosition(launcherPos));
        }
    }

    public Command reset() {
        launchStage = false;
        return new InstantCommand(()->launcherHeightServo.setPosition(IDLE));
    }
}
