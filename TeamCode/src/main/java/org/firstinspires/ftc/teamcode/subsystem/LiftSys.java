package org.firstinspires.ftc.teamcode.subsystem;

import android.util.Log;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.*;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleSupplier;


@Config
public class LiftSys extends SubsystemBase {
    private final MotorEx left, right;

    private double highestVoltage = 13.3;

    public static double NONE = -5;
    public static double LOW = 350;
    public static double MID = 500;
    public static double HIGH = 650;

    public static double kp = 0.03;
    public static double kd = 0.00001;
    public static double ki = 0;
    public static double kg = 0.11;
    public static double kgMax = 0.2;
    public static double kgMin = 0;

    public static double voltageProportion = 0;

    public static double maxVel = 1000;
    public static double maxAccel = 2000;

    public static double targetHeight = 0;

    public static int threshold = 5;

    public static double manualUpPower = 20;
    public static double manualDownPower = 20;

    private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(maxVel, maxAccel);

    private final ProfiledPIDController rightProfiledController = new ProfiledPIDController(kp, ki, kd, constraints);
    private final ProfiledPIDController leftProfiledController = new ProfiledPIDController(kp, ki, kd, constraints);


    private final PIDController leftController = new PIDController(kp, ki, kd);
    private final PIDController rightController = new PIDController(kp, ki, kd);

    private final TouchSensor limitSwitch;

    private ElapsedTime voltageTimer;
    private VoltageSensor voltageSensor;
    private double voltage;

    private final DoubleSupplier manualPower;
    private boolean manual = false;

    public LiftSys(MotorEx left, MotorEx right, TouchSensor sensor, HardwareMap.DeviceMapping<VoltageSensor> voltageSensor
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

    public void setHeight(double height) {
        targetHeight = height;
    }


    public Command goTo(double height) {
        return new InstantCommand(() -> {
            setHeight(height);
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

    public double getPosErrorL() {
        return leftProfiledController.getPositionError();
    }

    public double getSetPointL() {
        return leftProfiledController.getSetpoint().position;
    }

    public double getVelocityL() {
        return leftProfiledController.getSetpoint().velocity;
    }

    public double getProfilePowerL() {
        return (leftProfiledController.calculate(right.getCurrentPosition(), targetHeight)) + kg;
    }

    public double getNormalPIDOutput() {
        return leftProfiledController.calculate(right.getCurrentPosition(), targetHeight) + kg;
    }

    public double getPowerL() {
        return left.get();
    }

    public Command manualSetHeight(DoubleSupplier power) {
        return new RunCommand(() -> {
            if (Math.abs(power.getAsDouble()) > 0.01) {
                Log.d("asd", "getting power");
                if (power.getAsDouble() < 0) {
//                    targetHeight -= power.getAsDouble() * manualDownPower;
                    right.set(power.getAsDouble() * manualDownPower);
                    left.set(power.getAsDouble() * manualDownPower);
                } else {
//                    targetHeight -= power.getAsDouble() * manualUpPower;
                    right.set(power.getAsDouble() * manualUpPower);
                    left.set(power.getAsDouble() * manualUpPower);
                }
                targetHeight = left.getCurrentPosition();
            }
        }, this);
    }


    @Override
    public void periodic() {
        if (limitSwitch.isPressed()) {
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
//                targetHeight -= manualPower.getAsDouble() * manualDownPower;
                right.set(manualPower.getAsDouble());
                left.set(manualPower.getAsDouble());
                Log.d("asd", "" + manualPower.getAsDouble() * manualDownPower);
            } else {
//                targetHeight -= manualPower.getAsDouble() * manualUpPower;
                right.set(manualPower.getAsDouble());
                left.set(manualPower.getAsDouble());
            }
//            targetHeight = (left.getCurrentPosition() + right.getCurrentPosition()) / 2.0;
            manual = true;
        } else if (manual) {
            Log.d("asd", "doing kg");
            right.set(kg);
            left.set(kg);
        } else {
            double kgStored = kg;
            if (left.getCurrentPosition() > 520)
                kg = kgStored + ((kgMax - kgStored) / (730 - 520)) * (left.getCurrentPosition() - 520);
            if (left.getCurrentPosition() < 100)
                kg = kgStored - ((kgStored - kgMin) / (100)) * (100 - left.getCurrentPosition());

            right.set((rightProfiledController.calculate(right.getCurrentPosition(), targetHeight) + kg) * (13.3 / voltage));
            left.set((leftProfiledController.calculate(left.getCurrentPosition(), targetHeight) + kg) * (13.3 / voltage));
        }


        double voltReading = voltageSensor.getVoltage();
        if (voltReading > highestVoltage) {
            highestVoltage = voltReading;
        }

    }
}
