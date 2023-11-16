package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import org.firstinspires.ftc.teamcode.util.commands.CycleCommand;

@Config
public class BoxSubsystem extends SubsystemBase {
    public static double STOP_FIST = 0.7;
    public static double STOP_SECOND = 0.55;
    public static double RELEASE_FIRST = 0.5;
    public static double RELEASE_SECOND = 0.7;

    private final ServoEx first;
    private final ServoEx second;

    public BoxSubsystem(ServoEx first, ServoEx second) {
        this.first = first;
        this.second = second;
    }

    public Command cycle() {
        return new CycleCommand(
                new InstantCommand(() -> {
                    first.setPosition(STOP_FIST);
                    second.setPosition(STOP_SECOND);
                }),
                new InstantCommand(() -> first.setPosition(RELEASE_FIRST)),
                new InstantCommand(() -> second.setPosition(RELEASE_SECOND))
        );
    }

    public Command stopFirst() {
        return new InstantCommand(() -> first.setPosition(STOP_FIST));
    }

    public Command releaseFirst() {
        return new InstantCommand(() -> first.setPosition(RELEASE_FIRST));
    }

    public Command stopSecond() {
        return new InstantCommand(() -> second.setPosition(STOP_SECOND));
    }

    public Command releaseSecond() {
        return new InstantCommand(() -> second.setPosition(RELEASE_SECOND));
    }

    public double getFirstPosition() {
        return first.getPosition();
    }

    public double getSecondPosition() {
        return second.getPosition();
    }
}
