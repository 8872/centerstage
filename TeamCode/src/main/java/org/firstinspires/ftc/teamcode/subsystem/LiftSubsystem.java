package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.*;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

@Config
public class LiftSubsystem extends SubsystemBase {
    public static double NONE = 10;
    public static double FIRST = -325;
    public static double SECOND = -525;
    public static double THIRD = -725;

    public static double kg = -0.11;
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

    public static double manualUpPower = 80;
    public static double manualDownPower = 80;


    private final TouchSensor limitSwitchL;
    private final TouchSensor limitSwitchR;

    public LiftSubsystem(MotorEx left, MotorEx right, TouchSensor touchSensorL, TouchSensor touchSensorR) {
        this.left = left;
        this.right = right;
        this.limitSwitchL = touchSensorL;
        this.limitSwitchR = touchSensorR;
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

    public Command manualSetHeight(DoubleSupplier power){
        return new RunCommand(() -> {
            if(Math.abs(power.getAsDouble()) > 0.01) {
                if(power.getAsDouble() < 0){
                    leftController.setGoal((left.getCurrentPosition()+(power.getAsDouble()*manualUpPower)));
                    rightController.setGoal(right.getCurrentPosition()+(power.getAsDouble()*manualUpPower));
                }else{
                    leftController.setGoal((left.getCurrentPosition()+(power.getAsDouble()*manualDownPower)));
                    rightController.setGoal(right.getCurrentPosition()+(power.getAsDouble()*manualDownPower));
                }
            }
        }, this);
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
        double leftOutput = leftController.calculate(left.getCurrentPosition()) + kg;
        double rightOutput = rightController.calculate(right.getCurrentPosition()) + kg;
        left.set(leftOutput);
        right.set(rightOutput);
    }
}