package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
@Config
public class LauncherSys extends SubsystemBase {
    SimpleServo launcherHeightServo;
    SimpleServo launcherServo;

    public static double launcherPos = 0.5;
    public static double launcherHeightPos = 0.5;
    public LauncherSys(SimpleServo launcherHeightServo, SimpleServo launcherServo) {
        this.launcherHeightServo = launcherHeightServo;
        this.launcherServo = launcherServo;
    }

    public Command release() {return new InstantCommand(this::launch);}

    public void launch() {
        launcherServo.setPosition(0.5);
        launcherHeightServo.setPosition(0.5);
    }
}
