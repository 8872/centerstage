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

import java.util.function.DoubleSupplier;

@Config
public class LiftSubsystem extends SubsystemBase {
    public static double NONE = 0;
    public static double FIRST = -180;
    public static double SECOND = -360;
    public static double THIRD = -540;
    public static double MAX = -719;

    public static double kg = -0.04;
    public static double kp = 0.01;
    public static double ki = 0.0;
    public static double kd = 0.0001;

    public static double maxVelUp = 16000;
    public static double maxAccelUp = 16000;

    public static double maxVelDown = 8000;
    public static double maxAccelDown = 3000;

    private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(maxVelUp, maxAccelUp);
    private final ProfiledPIDController leftController = new ProfiledPIDController(kp, ki, kd,
            constraints);
    private final ProfiledPIDController rightController = new ProfiledPIDController(kp, ki, kd,
            constraints);

    public static int threshold = 1;

    private double targetHeight;

    private final MotorEx left;
    private final MotorEx right;

    private final DoubleSupplier doubleSupplier;

    private final TouchSensor limitSwitchL;
    private final TouchSensor limitSwitchR;

    public LiftSubsystem(MotorEx left, MotorEx right, TouchSensor touchSensorL,TouchSensor touchSensorR,DoubleSupplier doubleSupplier) {
        this.left = left;
        this.right = right;
        this.limitSwitchL = touchSensorL;
        this.limitSwitchR = touchSensorR;
        this.doubleSupplier = doubleSupplier;
    }

    public void setHeight(double height) {
        if (height < targetHeight) {
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

    public boolean getOverCurrentL() {
        return left.motorEx.isOverCurrent() || right.motorEx.isOverCurrent();
    }

    @Override
    public void periodic() {
//        if (doubleSupplier.getAsDouble() != 0) {
//            setHeight((doubleSupplier.getAsDouble()+1)*(MAX)/2);
//        } else {
//            double leftOutput = leftController.calculate(left.getCurrentPosition()) + kg;
//            double rightOutput = rightController.calculate(right.getCurrentPosition()) + kg;
//            left.set(leftOutput);
//            right.set(rightOutput);
//        }
        if (limitSwitchL.isPressed() || limitSwitchR.isPressed()) {
            left.resetEncoder();
            right.resetEncoder();
        }
        double leftOutput = leftController.calculate(left.getCurrentPosition()) + kg;
        double rightOutput = rightController.calculate(right.getCurrentPosition()) + kg;
        left.set(leftOutput);
        right.set(rightOutput);
    }
}