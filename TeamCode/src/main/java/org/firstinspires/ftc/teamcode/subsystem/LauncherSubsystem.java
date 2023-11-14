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

    public LauncherSubsystem(SimpleServo launcherHeightServo, SimpleServo launcherServo) {
        this.launcherHeightServo = launcherHeightServo;
        this.launcherServo = launcherServo;
    }

    public Command release() {
        return new InstantCommand(() -> {
            launcherServo.setPosition(launcherPos);
            launcherHeightServo.setPosition(launcherHeightPos);
        });
    }
}