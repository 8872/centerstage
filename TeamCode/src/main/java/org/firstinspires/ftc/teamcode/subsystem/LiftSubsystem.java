package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

@Config
public class LiftSubsystem extends SubsystemBase {
    public static double NONE = 0;
    public static double FIRST = -300;
    public static double SECOND = -400;
    public static double THIRD = -570;
    public static double MAX = -719;

    public static double kg = -0.04;
    public static double kp = 0.013;
    public static double ki = 0.0;
    public static double kd = 0.0000;

    public static double maxVelUp = 16000;
    public static double maxAccelUp = 16000;

    public static double maxVelDown = 1000;
    public static double maxAccelDown = 1000;

    private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(maxVelUp, maxAccelUp);
    private final ProfiledPIDController leftController = new ProfiledPIDController(kp, ki, kd,
            constraints);
    private final ProfiledPIDController rightController = new ProfiledPIDController(kp, ki, kd,
            constraints);

    public static int threshold = 10;

    private double targetHeight;

    private final MotorEx left;
    private final MotorEx right;

    private final DoubleSupplier doubleSupplier;
    private final BooleanSupplier booleanSupplier;
    private final TouchSensor limitSwitchL;
    private final TouchSensor limitSwitchR;

    public LiftSubsystem(MotorEx left, MotorEx right, TouchSensor touchSensorL, TouchSensor touchSensorR, DoubleSupplier doubleSupplier, BooleanSupplier booleanSupplier) {
        this.left = left;
        this.right = right;
        this.limitSwitchL = touchSensorL;
        this.limitSwitchR = touchSensorR;
        this.doubleSupplier = doubleSupplier;
        this.booleanSupplier = booleanSupplier;
    }

    public void setHeight(double height) {
        if (height > targetHeight) {
            constraints = new TrapezoidProfile.Constraints(maxVelDown, maxAccelDown);
        } else {
            constraints = new TrapezoidProfile.Constraints(maxVelUp, maxAccelUp);
        }
        leftController.setConstraints(constraints);
        rightController.setConstraints(constraints);
        targetHeight = height;

        leftController.setGoal(height);
        rightController.setGoal(height);
    }

    public Command goTo(double height) {
        return new InstantCommand(() -> setHeight(height))
                .andThen(new WaitUntilCommand(this::atTarget));
    }

    public boolean atTarget() {
        return right.getCurrentPosition() < targetHeight + threshold && right.getCurrentPosition() > targetHeight - threshold
                || left.getCurrentPosition() < targetHeight + threshold && left.getCurrentPosition() > targetHeight - threshold;
    }

    public double getTargetHeight() {
        return targetHeight;
    }

    public boolean getOverCurrent() {
        return left.motorEx.isOverCurrent() || right.motorEx.isOverCurrent();
    }

    @Override
    public void periodic() {

        if (limitSwitchL.isPressed() || limitSwitchR.isPressed() ) {
            left.resetEncoder();
            right.resetEncoder();
        }
        double leftOutput = leftController.calculate(left.getCurrentPosition()) + Math.cos(Math.toRadians(targetHeight / 145.6)) * kg;
        double rightOutput = rightController.calculate(right.getCurrentPosition()) + Math.cos(Math.toRadians(targetHeight / 145.6)) * kg;
        left.set(leftOutput);
        right.set(rightOutput);
    }
}