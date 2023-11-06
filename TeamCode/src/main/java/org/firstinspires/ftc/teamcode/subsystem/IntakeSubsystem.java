package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.util.PixelColor;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

@Config
public class IntakeSubsystem extends SubsystemBase {
    public static double currentThreshold = 1000;
    public static double LOW = 0.05;
    public static double HIGH = 0.4;
    public static double IN = 0.7;
    public static double OUT = -1;
    public static double STOP = 0;
    public static double PINK_THRESH = 0;
    public static double GREEN_THRESH = 0;
    public static double YELLOW_THRESH = 0;

    private BooleanSupplier fpower;
    private BooleanSupplier rpower;
    private final MotorEx intake;
    private final ServoEx stack;
    private final ColorSensor color;

    public IntakeSubsystem(MotorEx intake, ServoEx stack, ColorSensor color, BooleanSupplier fpower, BooleanSupplier rpower) {
        this.intake = intake;
        this.stack = stack;
        this.color = color;
        this.fpower = fpower;
        this.rpower = rpower;
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

    @Override
    public void periodic() {
        if(fpower.getAsBoolean()) {
            intake.set(IN);
            stack.setPosition(LOW);
        } else if (rpower.getAsBoolean()) {
            intake.set(OUT);
            stack.setPosition(LOW);
        } else {
            stack.setPosition(HIGH);
            intake.set(0);
        }
    }
}
