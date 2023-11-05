package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.util.PixelColor;

@Config
public class IntakeSubsystem extends SubsystemBase {
    public static double PINK_THRESH = 0;
    public static double GREEN_THRESH = 0;
    public static double YELLOW_THRESH = 0;

    public static double LOW = 0.1;
    public static double MEDIUM = 0.2;
    public static double HIGH = 0.3;

    public static double IN = 1;
    public static double OUT = -1;
    public static double STOP = 0;

    private final MotorEx intake;
    private final ServoEx stack;
    private final ColorSensor color;

    public IntakeSubsystem(MotorEx intake, ServoEx stack, ColorSensor color) {
        this.intake = intake;
        this.stack = stack;
        this.color = color;
    }

    public PixelColor getPixelColor() {
        if (color.red() > PINK_THRESH) {
            return PixelColor.PINK;
        } else if (color.green() > GREEN_THRESH) {
            return PixelColor.GREEN;
        } else if (color.red() > YELLOW_THRESH) {
            return PixelColor.YELLOW;
        }
        return PixelColor.WHITE;
    }

    public Command in() {
        return new InstantCommand(() -> intake.set(IN));
    }

    public Command out() {
        return new InstantCommand(() -> intake.set(OUT));
    }

    public Command stop() {
        return new InstantCommand(() -> intake.set(STOP));
    }

    public Command setHeight(double height) {
        return new InstantCommand(() -> stack.setPosition(height));
    }

    public double getCurrentHeight() {
        return stack.getPosition();
    }

    public double getPower() {
        return intake.get();
    }
}
