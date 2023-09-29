package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class ClawSys extends SubsystemBase {
    /**
     * 1 servo
     */
    ServoEx clawServo;
    ColorSensor colorSensor;
    public enum ClawPosition {
        GRAB(0.0),
        FIRST(0.0),
        SECOND(0.0);
        public final double pos;
        ClawPosition(double pos) {
            this.pos = pos;
        }
    }
    public ClawPosition clawPos;
    public enum PixelColor {
        NONE(0),
        RED(0),
        GREEN(0),
        BLUE(0);
        public final int colorThreshold;
        PixelColor(int colorThreshold) {
            this.colorThreshold = colorThreshold;
        }
    }

    public ClawSys(ServoEx clawServo, ColorSensor colorSensor) {
        this.clawServo = clawServo;
        this.colorSensor = colorSensor;
    }

    public void claw(ClawPosition position) {
        clawServo.setPosition(position.pos);
        clawPos = position;
    }
    public PixelColor getPixelColor() {
        if (LiftSys.isDown()) {
            if (colorSensor.red() > PixelColor.RED.colorThreshold) {
                return PixelColor.RED;
            } else if (colorSensor.green() > PixelColor.GREEN.colorThreshold) {
                return PixelColor.GREEN;
            } else if (colorSensor.blue() > PixelColor.BLUE.colorThreshold) {
                return PixelColor.BLUE;
            }
        }
        return PixelColor.NONE;
    }
    public ClawPosition getClaw() {
        return clawPos;
    }

    public double getClawPos() {
        return clawServo.getPosition();
    }
}
