package org.firstinspires.ftc.teamcode.subsystem;

import android.util.Log;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleSupplier;


@Config
public class LiftSubsystem extends SubsystemBase {
    private final MotorEx left, right;
    private final TouchSensor limitSwitch;

    public static double NONE = 0;
    public static double LOW = 925;
    public static double MID = 1300;
    public static double HIGH = 1800;
    public static double JAM = 20;

    public static double kp = 0.009;
    public static double kd = 0.0001;
    public static double ki = 0.0003;
    public static double kg = 0;

    public static double targetHeight = 0;

    public static int threshold = 5;

    private final PIDController leftController = new PIDController(kp, ki, kd);
    private final PIDController rightController = new PIDController(kp, ki, kd);

    private final ElapsedTime voltageTimer;
    private final VoltageSensor voltageSensor;
    private double voltage;
    private double highestVoltage = 13.3;

    private final DoubleSupplier manualPower;
    private boolean manual = false;

    public LiftSubsystem(MotorEx left, MotorEx right, TouchSensor sensor, HardwareMap.DeviceMapping<VoltageSensor> voltageSensor
            , DoubleSupplier manualPower) {
        this.left = left;
        this.right = right;
        this.left.resetEncoder();
        this.right.resetEncoder();
        this.limitSwitch = sensor;
        this.voltageTimer = new ElapsedTime();
        this.voltageTimer.reset();
        this.voltageSensor = voltageSensor.iterator().next();
        this.voltage = this.voltageSensor.getVoltage();
        this.manualPower = manualPower;
        targetHeight = 0;
    }

    public Command goTo(double height) {
        return new InstantCommand(() -> {
            targetHeight = height;
            manual = false;
        });
    }

    public boolean atTarget() {
        return right.getCurrentPosition() < targetHeight + threshold && right.getCurrentPosition() > targetHeight - threshold
                || left.getCurrentPosition() < targetHeight + threshold && left.getCurrentPosition() > targetHeight - threshold;
    }

    public double getVoltage() {
        return voltage;
    }


    public double getPowerL() {
        return left.get();
    }

    @Override
    public void periodic() {
        Log.d("asd", ""+right.getCurrentPosition());
        if (limitSwitch.isPressed()) {
            Log.d("asd", "was pressed");
            left.resetEncoder();
            right.resetEncoder();
        }

        if (voltageTimer.seconds() > 15) {
            voltage = highestVoltage;
            highestVoltage = 13.3;
            voltageTimer.reset();
        }

        if (Math.abs(manualPower.getAsDouble()) > 0.01) {
            if (manualPower.getAsDouble() > 0) {
                right.set(manualPower.getAsDouble());
                left.set(manualPower.getAsDouble());
            } else {
                right.set(manualPower.getAsDouble());
                left.set(manualPower.getAsDouble());
            }
            manual = true;
        } else if (manual) {
            right.set(kg);
            left.set(kg);
        } else {
            // scuffed hack, runs full power at the jam height, otherwise just does pid
            if (right.getCurrentPosition() < JAM && targetHeight == NONE) {
                if (!limitSwitch.isPressed()) {
                    left.set(-1);
                    right.set(-1);
                } else {
                    right.set((rightController.calculate((right.getCurrentPosition()), targetHeight) + kg) * (13.3 / voltage));
                    left.set((leftController.calculate(right.getCurrentPosition(), targetHeight) + kg) * (13.3 / voltage));
                }
            } else {
                right.set((rightController.calculate((right.getCurrentPosition()), targetHeight) + kg) * (13.3 / voltage));
                left.set((leftController.calculate(right.getCurrentPosition(), targetHeight) + kg) * (13.3 / voltage));
            }
        }

        double voltReading = voltageSensor.getVoltage();
        if (voltReading > highestVoltage) {
            highestVoltage = voltReading;
        }
    }
}
