package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
@Config
public class BoxSys extends SubsystemBase {
    public static boolean nah = false;
    public static final double INNER_SERVO_LOCKED = 0.8;
    public static final double INNER_SERVO_UNLOCKED = 0.5;
    public static final double OUTER_SERVO_LOCKED = 0.5;
    public static final double OUTER_SERVO_UNLOCKED = 0.8;
    private SimpleServo innerServo;
    private SimpleServo outerServo;
    public enum BoxState {
        CLOSED, /* both servos are locked */
        INNER, /* only the inner servo is locked */
        INTAKE /* both servos are unlocked */
    }
    public static BoxState boxState = BoxState.INTAKE;
    public BoxSys(SimpleServo innerServo, SimpleServo outerServo) {
        this.innerServo = innerServo;
        this.outerServo = outerServo;
    }

    public Command intake() {
        return new InstantCommand(() -> {
            boxState = BoxState.INTAKE;
            innerServo.setPosition(INNER_SERVO_UNLOCKED);
            outerServo.setPosition(OUTER_SERVO_UNLOCKED);
        }, this);
    }

    public void release() {
        if(outerServo.getPosition() == OUTER_SERVO_UNLOCKED) {
            boxState = BoxState.INTAKE;
            innerServo.setPosition(INNER_SERVO_UNLOCKED);
        } else {
            boxState = BoxState.INNER;
            outerServo.setPosition(OUTER_SERVO_UNLOCKED);
            innerServo.setPosition(INNER_SERVO_LOCKED);
            nah = true;
        }
    }

    public Command close() {
        return new InstantCommand(() -> {
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
