package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

@Config
public class BoxSubsystem extends SubsystemBase {
    public static boolean nah = false;
    public static final double INNER_SERVO_LOCKED = 0.8;
    public static final double INNER_SERVO_UNLOCKED = 0.5;
    public static final double OUTER_SERVO_LOCKED = 0.5;
    public static final double OUTER_SERVO_UNLOCKED = 0.8;
    private final SimpleServo innerServo;
    private final SimpleServo outerServo;
    private BlinkinSubsystem blinkinSubsystem;

    public enum BoxState {
        CLOSED, /* both servos are locked */
        INNER, /* only the inner servo is locked */
        INTAKE /* both servos are unlocked */
    }

    public static BoxState boxState = BoxState.INTAKE;

    public BoxSubsystem(SimpleServo innerServo, SimpleServo outerServo, BlinkinSubsystem blinkinSubsystem) {
        this.innerServo = innerServo;
        this.outerServo = outerServo;
        this.blinkinSubsystem = blinkinSubsystem;
    }

    public BoxSubsystem(SimpleServo innerServo, SimpleServo outerServo) {
        this.innerServo = innerServo;
        this.outerServo = outerServo;
    }

    public Command intake() {
        return new InstantCommand(() -> {
            blinkinSubsystem.setCurrentPattern(null);
            boxState = BoxState.INTAKE;
            innerServo.setPosition(INNER_SERVO_UNLOCKED);
            outerServo.setPosition(OUTER_SERVO_UNLOCKED);
        }, this);
    }

    public void release() {
        if (outerServo.getPosition() == OUTER_SERVO_UNLOCKED) {
            boxState = BoxState.INTAKE;
            innerServo.setPosition(INNER_SERVO_UNLOCKED);
        } else {
            blinkinSubsystem.setCurrentPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            boxState = BoxState.INNER;
            outerServo.setPosition(OUTER_SERVO_UNLOCKED);
            innerServo.setPosition(INNER_SERVO_LOCKED);
            nah = true;
        }
    }

    public Command close() {
        return new InstantCommand(() -> {
            blinkinSubsystem.setCurrentPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
            boxState = BoxState.CLOSED;
            innerServo.setPosition(INNER_SERVO_LOCKED);
            outerServo.setPosition(OUTER_SERVO_LOCKED);
        }, this);
    }

    public double getInnerServo() {
        return innerServo.getPosition();
    }

    public double getOuterServo() {
        return outerServo.getPosition();
    }
}
