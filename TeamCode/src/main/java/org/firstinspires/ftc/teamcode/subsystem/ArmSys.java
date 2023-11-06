package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
@Config
public class ArmSys extends SubsystemBase {
    public static double pitchHome = 0.43;
    public static double pitchAway = 1;

    public static double armHome = 0.8;
    public static double armAway = 0.35;

    private final SimpleServo pitchServo;
    private final SimpleServo armServo;

    public ArmSys(SimpleServo pitchServo, SimpleServo armServo) {
        this.armServo = armServo;
        this.pitchServo = pitchServo;
    }

    public Command pitchHome() {return new InstantCommand(() -> pitchServo.setPosition(pitchHome));}

    public Command pitchAway() {return new InstantCommand(() -> pitchServo.setPosition(pitchAway));}

    public Command armHome() {return new InstantCommand(()->armServo.setPosition(armHome));}

    public Command armAway() {return new InstantCommand(()->armServo.setPosition(armAway));}

    public Command away() {return new InstantCommand(this::push);}

    public Command home() {return new InstantCommand(this::pull);}

    public void push() {
        armServo.setPosition(armAway);
        pitchServo.setPosition(pitchAway);
    }

    public void pull() {
        armServo.setPosition(armHome);
        pitchServo.setPosition(pitchHome);
    }
}
