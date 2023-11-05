package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.*;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;

@Config
public class LiftSubsystem extends SubsystemBase {

    public static double NONE = 0;
    public static double FIRST = 0;
    public static double SECOND = 0;
    public static double THIRD = 0;

    public static double kp = 0.0;
    public static double ki = 0.0;
    public static double kd = 0.0;
    public static double kg = 0.0;

    public static double maxVel = 0;
    public static double maxAccel = 0;
    private final ProfiledPIDController leftController = new ProfiledPIDController(kp, ki, kd,
            new TrapezoidProfile.Constraints(maxVel, maxAccel));
    private final ProfiledPIDController rightController = new ProfiledPIDController(kp, ki, kd,
            new TrapezoidProfile.Constraints(maxVel, maxAccel));

    public static int threshold = 0;

    private double targetHeight;

    private final MotorEx left;
    private final MotorEx right;

    public LiftSubsystem(MotorEx left, MotorEx right) {
        this.left = left;
        this.right = right;
        setHeight(NONE);
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
        return right.getCurrentPosition() < targetHeight + threshold
                && right.getCurrentPosition() > targetHeight - threshold;
    }

    public double getTargetHeight() {
        return targetHeight;
    }

    @Override
    public void periodic() {
        double leftOutput = leftController.calculate(left.getCurrentPosition()) + kg;
        double rightOutput = rightController.calculate(right.getCurrentPosition()) + kg;
        left.set(leftOutput);
        right.set(rightOutput);
    }
}
