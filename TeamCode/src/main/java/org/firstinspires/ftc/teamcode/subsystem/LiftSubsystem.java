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

    public static double maxVel = 8000;
    public static double maxAccel = 3000;
    private final ProfiledPIDController leftController = new ProfiledPIDController(kp, ki, kd,
            new TrapezoidProfile.Constraints(maxVel, maxAccel));
    private final ProfiledPIDController rightController = new ProfiledPIDController(kp, ki, kd,
            new TrapezoidProfile.Constraints(maxVel, maxAccel));

    public static int threshold = 1;

    private double targetHeight;

    private final MotorEx left;
    private final MotorEx right;

    private final DoubleSupplier doubleSupplier;

    private TouchSensor limitSwitchL, limitSwitchR;

    public LiftSubsystem(MotorEx left, MotorEx right, TouchSensor touchSensorL,TouchSensor touchSensorR,DoubleSupplier doubleSupplier) {
        this.left = left;
        this.right = right;
        this.limitSwitchL = touchSensorL;
        this.limitSwitchR = touchSensorR;
        this.doubleSupplier = doubleSupplier;
    }

    public void checkLimitSwitch() {
        if (limitSwitchL.isPressed() || limitSwitchR.isPressed()) {
            left.resetEncoder();
            right.resetEncoder();
        }
    }


    public void setHeight(double height) {
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
        checkLimitSwitch();
        double leftOutput = leftController.calculate(left.getCurrentPosition()) + kg;
        double rightOutput = rightController.calculate(right.getCurrentPosition()) + kg;
        left.set(leftOutput);
        right.set(rightOutput);
    }
}