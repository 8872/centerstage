package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class ClawSys {
    /**
     * 1 servo
     */
    ServoEx clawServo;
    ColorSensor colorSensor;
    public enum ClawPosition {
        FIRST(0.0),
        SECOND(1.0);
        public final double pos;
        ClawPosition(double pos) {
            this.pos = pos;
        }
    }

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
    }

    public void claw(ClawPosition position) {
        clawServo.setPosition(position.pos);
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
}
