package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;

@Config
public class ArmSubsystem extends SubsystemBase {
    public static double armIntake = 0.75;
    public static double armOuttake = 0.23;
    public static double pitchIntake = 0.3;
    public static double pitchOuttake = 0.77;
    public SimpleServo armServo;
    public SimpleServo pitchServo;

    public enum ArmState {
        INTAKE,
        DEPOSIT
    }

    public static ArmState armState = ArmState.INTAKE;

    public ArmSubsystem(SimpleServo armServo, SimpleServo pitchServo) {
        this.armServo = armServo;
        this.pitchServo = pitchServo;
    }

    public Command intake() {
        return new InstantCommand(() -> {
            armState = ArmState.INTAKE;
            armServo.setPosition(armIntake);
            pitchServo.setPosition(pitchIntake);
        }, this);
    }

    public Command deposit() {
        return new InstantCommand(() -> {
            armState = ArmState.DEPOSIT;
            armServo.setPosition(armOuttake);
            pitchServo.setPosition(pitchOuttake);
        }, this);
    }
}

