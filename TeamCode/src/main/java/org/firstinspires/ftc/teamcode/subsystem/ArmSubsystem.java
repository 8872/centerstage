package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;

@Config
public class ArmSubsystem extends SubsystemBase {
    public static double INTAKE = 0;
    public static double DEPOSIT = 1;

    private final ServoEx left;
    private final ServoEx right;

    public ArmSubsystem(ServoEx left, ServoEx right) {
        this.left = left;
        this.right = right;
    }

    public Command intake() {
        return new InstantCommand(() -> setPosition(INTAKE));
    }

    public Command deposit() {
        return new InstantCommand(() -> setPosition(DEPOSIT));
    }

    private void setPosition(double pos) {
        left.setPosition(pos);
        right.setPosition(pos);
    }

    public double getPosition() {
        return left.getPosition();
    }
}
