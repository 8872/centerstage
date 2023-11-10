package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
@Config
public class ClawSys extends SubsystemBase {

    public static double STOP = 1;
    public static double RELEASE = 0.5;

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

    public void cycle() {
        switch (clawState) {
            case ALL:
                first.setPosition(STOP);
                second.setPosition(STOP);
                clawState = ClawState.FIRST;
            case FIRST:
                first.setPosition(RELEASE);
                clawState = ClawState.SECOND;
            case SECOND:
                second.setPosition(RELEASE);
                clawState = ClawState.ALL;
        }
    }

    public Command release() {return new InstantCommand(this::cycle);}

    public Command stopFirst() {
        return new InstantCommand(() -> first.setPosition(STOP));
    }

    public Command releaseFirst() {
        return new InstantCommand(() -> first.setPosition(RELEASE));
    }

    public Command stopSecond() {
        return new InstantCommand(() -> second.setPosition(RELEASE));
    }

    public Command releaseSecond() {
        return new InstantCommand(() -> second.setPosition(RELEASE));
    }

    public double getFirstPosition() {
        return first.getPosition();
    }

    public double getSecondPosition() {
        return second.getPosition();
    }

}
