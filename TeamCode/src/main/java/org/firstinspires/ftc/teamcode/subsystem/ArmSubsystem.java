package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;

@Config
public class ArmSubsystem extends SubsystemBase {
    public static double PITCH_INTAKE = 0.37;
    public static double PITCH_DEPOSIT = 1;

    public static double ARM_INTAKE = 0.78;
    public static double ARM_DEPOSIT = 0.35;

    private final SimpleServo pitch;
    private final SimpleServo arm;

    public ArmSubsystem(SimpleServo pitch, SimpleServo arm) {
        this.pitch = pitch;
        this.arm = arm;
    }

    public Command deposit() {
        return new InstantCommand(() -> {
            pitch.setPosition(PITCH_DEPOSIT);
            arm.setPosition(ARM_DEPOSIT);
        });
    }

    public Command intake() {
        return new InstantCommand(() -> {
            pitch.setPosition(PITCH_INTAKE);
            arm.setPosition(ARM_INTAKE);
        });
    }
}
