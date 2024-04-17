package org.firstinspires.ftc.teamcode.subsystem;

import android.util.Log;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.Precision;

import java.util.function.DoubleSupplier;

@Config
public class IntakeSubsystem extends SubsystemBase {
    public static boolean algorithm = true;
    public static int spikeCount = 0;
    private boolean lastActive = false;
    private long spikeStartTime = 0;
    private MotorEx intake;
    private SimpleServo stack;
    private SimpleServo stack2;

    public static double intakeInPower = 0.67; // 0.7
    public static double intakeOutPower = 0.7;

    public static double servoHighPosition = 0.9; // 0.9
    public static double servoLowPosition = 0.7; // 0.7

    public static double servo2HighPosition = 0.6; // 0.6
    public static double servo2LowPosition = 0.4; // 0.4
    private final double[] coefficients;
    private final double[] coefficients2;

    private double highestVoltage = 13;


    private ElapsedTime voltageTimer;
    private VoltageSensor voltageSensor;
    private double voltage;
    private boolean hasSet = false;
    private BlinkinSubsystem blinkinSubsystem;

    public IntakeSubsystem(SimpleServo stack, SimpleServo stack2, MotorEx intake, HardwareMap.DeviceMapping<VoltageSensor> voltageSensor, BlinkinSubsystem blinkinSubsystem) {
        this.stack = stack;
        this.stack2 = stack2;
        this.intake = intake;
        this.blinkinSubsystem = blinkinSubsystem;
        coefficients = Precision.calculateSlopeAndIntercept(0, servoHighPosition, 1, servoLowPosition);
        coefficients2 = Precision.calculateSlopeAndIntercept(0, servo2HighPosition, 1, servo2LowPosition);
        Log.d("asd", "set stack");
        stack.setPosition(servoHighPosition);
        stack2.setPosition(servo2HighPosition);

        this.voltageTimer = new ElapsedTime();
        this.voltageTimer.reset();
        this.voltageSensor = voltageSensor.iterator().next();
        this.voltage = this.voltageSensor.getVoltage();
    }

    public IntakeSubsystem(SimpleServo stack, SimpleServo stack2, MotorEx intake, HardwareMap.DeviceMapping<VoltageSensor> voltageSensor) {
        this.stack = stack;
        this.stack2 = stack2;
        this.intake = intake;
        coefficients = Precision.calculateSlopeAndIntercept(0, servoHighPosition, 1, servoLowPosition);
        coefficients2 = Precision.calculateSlopeAndIntercept(0, servo2HighPosition, 1, servo2LowPosition);
        stack.setPosition(servoHighPosition);
        stack2.setPosition(servo2HighPosition);

        this.voltageTimer = new ElapsedTime();
        this.voltageTimer.reset();
        this.voltageSensor = voltageSensor.iterator().next();
        this.voltage = this.voltageSensor.getVoltage();
    }


    public double getCurrent() {
        return intake.motorEx.getCurrent(CurrentUnit.MILLIAMPS);
    }

    public Command intake(DoubleSupplier fpower, DoubleSupplier rpower) {
        return new RunCommand(() -> {
            if (fpower.getAsDouble() != 0) {
                blinkinSubsystem.setCurrentPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                hasSet = false;
                intake.set((13 / voltage) * intakeInPower);
                stack.setPosition((fpower.getAsDouble() * coefficients[0]) + coefficients[1]);
                stack2.setPosition((fpower.getAsDouble() * coefficients2[0]) + coefficients2[1]);
            } else if (rpower.getAsDouble() != 0) {
                blinkinSubsystem.setCurrentPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                hasSet = false;
                intake.set(-intakeOutPower);
                stack.setPosition((rpower.getAsDouble() * coefficients[0]) + coefficients[1]);
                stack2.setPosition((rpower.getAsDouble() * coefficients2[0]) + coefficients2[1]);
            } else {
                if(!hasSet) {
                    blinkinSubsystem.setCurrentPattern(null);
                    hasSet = true;
                }
                intake.set(0);
                stack.setPosition(servoHighPosition);
                stack2.setPosition(servo2HighPosition);
            }
        }, this);
    }


    public Command runIntake(double power) {
        return new InstantCommand(() -> {
            intake.set(power);
        }, this);
    }

    public Command setStack1(double position) {
        return new InstantCommand(() -> {
            stack.setPosition(position);
        }, this);
    }

    public Command setStack2(double position) {
        return new InstantCommand(() -> {
            stack2.setPosition(position);
        }, this);
    }

    public double getIntakePower() {
        return intake.get();
    }

    @Override
    public void periodic() {
        if (voltageTimer.seconds() > 15) {
            voltage = highestVoltage;
            highestVoltage = 13;
            voltageTimer.reset();
        }

        double voltReading = voltageSensor.getVoltage();
        if (voltReading > highestVoltage) {
            highestVoltage = voltReading;
        }
    }
}
