package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.util.MathUtils;

import java.util.function.DoubleSupplier;

@Config
public class LauncherSubsystem extends SubsystemBase {
    private SimpleServo height;
    private SimpleServo release;

    public static double heightInitial = 0.65;
    public static double heightFinal = 0.4;
    public static int incrementFactor = 175;

    // TODO needs to be tuned!
    public static double releaseInitial = 0;
    public static double releaseFinal = 0.4;

    public LauncherSubsystem(SimpleServo height, SimpleServo release) {
        this.height = height;
        this.release = release;
        height.setPosition(heightInitial);
        release.setPosition(releaseInitial);
    }

    public Command move(DoubleSupplier increment) {
        return new RunCommand(() -> height.setPosition(MathUtils.clamp(
                height.getPosition() - increment.getAsDouble() / incrementFactor, heightFinal, heightInitial
        )), this);
    }

    public Command release() {
        return new InstantCommand(() -> release.setPosition(releaseFinal));
    }
}
