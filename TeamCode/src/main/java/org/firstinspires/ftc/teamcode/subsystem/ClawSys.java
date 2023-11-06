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

    public ClawSys(ServoEx first, ServoEx second) {
        this.first = first;
        this.second = second;
    }

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
