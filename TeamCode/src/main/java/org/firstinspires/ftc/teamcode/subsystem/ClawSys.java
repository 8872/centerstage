package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.*;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
@Config
public class ClawSys extends SubsystemBase {
    public static double stopFirst = 0.7;
    public static double stopSecond = 0.55;
    public static double releaseFirst = 0.5;
    public static double releaseSecond = 0.7;

    private final ServoEx first;
    private final ServoEx second;

    public enum ClawState {
        ALL,
        FIRST,
        SECOND
    }

    public ClawState clawState;

    public ClawSys(ServoEx first, ServoEx second) {
        this.first = first;
        this.second = second;
        this.clawState = ClawState.ALL;
    }

    public void cycleCommand() {
        ClawState state = clawState;
        switch (state) {
            case ALL:
                clawState = ClawState.FIRST;
                first.setPosition(stopFirst);
                second.setPosition(stopSecond);
                break;
            case FIRST:
                clawState = ClawState.SECOND;
                first.setPosition(releaseFirst);
                break;
            case SECOND:
                clawState = ClawState.ALL;
                second.setPosition(releaseSecond);
                break;
        }
    }
    public Command stopFirst() {
        return new InstantCommand(() -> first.setPosition(stopFirst));
    }

    public Command releaseFirst() {
        return new InstantCommand(() -> first.setPosition(releaseFirst));
    }

    public Command stopSecond() {
        return new InstantCommand(() -> second.setPosition(stopSecond));
    }

    public Command releaseSecond() {
        return new InstantCommand(() -> second.setPosition(releaseSecond));
    }
    public double getFirstPosition() {
        return first.getPosition();
    }

    public double getSecondPosition() {
        return second.getPosition();
    }

}
