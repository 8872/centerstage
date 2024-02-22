package org.firstinspires.ftc.teamcode.subsystem;

import android.util.Log;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
public class LiftSys extends SubsystemBase {
    private final MotorEx left, right;

    public static double NONE = -5;
    public static double LOW = 200;
    public static double MID = 425;
    public static double HIGH = 650;

    public static double dki = 0.0001; // 0
    public static double dkp = 0.0035; // 0.02
    public static double dkd = 0.0001;
    public static double dkg = 0.08;

    public static double uki = 0;
    public static double ukp = 0.04;
    public static double ukd = 0.0007;
    public static double ukg = 0.08;

    public static double jki = 0.0005;
    public static double jkp = 0.05;
    public static double jkd = 0;
    public static double jkg = 0.1;

    public static int jamHeight = 150;

    public static double kheight = 0;

    public static double upMaxVel = 4000;
    public static double upMaxAccel = 4000;
    public static double downMaxVel = 250; // 4000
    public static double downMaxAccel = 250; // 2000

    public static double targetHeight = 0;
    public static int posThreshold = 10;
    public static int velThreshold = 10;

    public static int threshold = 5;

    public static boolean profileDown = false;
    public static boolean profileUp = false;

    private TrapezoidProfile.Constraints upConstraints = new TrapezoidProfile.Constraints(upMaxVel, upMaxAccel);
    private TrapezoidProfile.Constraints downConstraints = new TrapezoidProfile.Constraints(downMaxVel, downMaxAccel);

    private final ProfiledPIDController upRightProfiledController = new ProfiledPIDController(ukp, uki, ukd, upConstraints);
    private final ProfiledPIDController upLeftProfiledController = new ProfiledPIDController(ukp, uki, ukd, upConstraints);
    private final ProfiledPIDController downLeftProfiledController = new ProfiledPIDController(dkp, dki, dkd, downConstraints);
    private final ProfiledPIDController downRightProfiledController = new ProfiledPIDController(dkp, dki, dkd, downConstraints);



    private final PIDController downLeftController = new PIDController(dkp, dki, dkd);
    private final PIDController downRightController = new PIDController(dkp, dki, dkd);
    private final PIDController upLeftController = new PIDController(ukp, uki, ukd);
    private final PIDController upRightController = new PIDController(ukp, uki, ukd);


    private final PIDController jamPIDLeft = new PIDController(jkp, jki, jkd);
    private final PIDController jamPIDRight = new PIDController(jkp, jki, jkd);

    private final TouchSensor limitSwitch;

    private ElapsedTime voltageTimer;
    private VoltageSensor voltageSensor;
    private double voltage;

    public LiftSys(MotorEx left, MotorEx right, TouchSensor sensor, HardwareMap.DeviceMapping<VoltageSensor> voltageSensor) {
        this.left = left;
        this.right = right;
        this.left.resetEncoder();
        this.right.resetEncoder();
        this.limitSwitch = sensor;
//        profiledRightController.setPID(kp, ki, kd);
//        profiledRightController.setTolerance(posThreshold, velThreshold);
//        profiledLeftController.setPID(kp, ki, kd);
//        profiledLeftController.setTolerance(posThreshold, velThreshold);
//        this.voltageTimer = new ElapsedTime();
//        this.voltageTimer.reset();
//        this.voltageSensor = voltageSensor.iterator().next();
//        this.voltage = this.voltageSensor.getVoltage();
    }

    public void setHeight(double height) {
        targetHeight = height;
    }

    public double getTargetHeight() {
        return targetHeight;
    }

    public Command goTo(double height) {
        return new InstantCommand(() -> setHeight(height))
                .andThen(new WaitUntilCommand(this::atTarget));
    }

    public boolean atTarget() {
        return right.getCurrentPosition() < targetHeight + threshold && right.getCurrentPosition() > targetHeight - threshold
                || left.getCurrentPosition() < targetHeight + threshold && left.getCurrentPosition() > targetHeight - threshold;
    }

    public double getVoltage() {
        return voltage;
    }

    public double getPosErrorL() {
        return downLeftProfiledController.getPositionError();
    }

    public double getSetPointL() {
        return downLeftProfiledController.getSetpoint().position;
    }

    public double getProfilePowerL() {
        return (downLeftProfiledController.calculate(right.getCurrentPosition(), targetHeight)) + dkg;
    }

    public double getNormalPIDOutput() {
        return downLeftProfiledController.calculate(right.getCurrentPosition(), targetHeight) + dkg;
    }

    public double getPowerL() {
        return left.get();
    }

    @Override
    public void periodic() {
        if (limitSwitch.isPressed()) {
            left.resetEncoder();
            right.resetEncoder();
        }
//        if (voltageTimer.seconds() > 5) {
//            voltage = voltageSensor.getVoltage();
//            voltageTimer.reset();
//        }
//        if(left.getCurrentPosition() == 0 && getTargetHeight() == 0) {
//            // do nothing
//        } else {
        double add = kheight * (targetHeight - 400) < 0 ? 0 : kheight * (targetHeight - 400);
//        if (profile) {
//            right.set((profiledRightController.calculate(right.getCurrentPosition(), targetHeight)) + kg + add);
//            left.set((profiledLeftController.calculate(left.getCurrentPosition(), targetHeight)) + kg + add);
//        } else {
//            right.set((rightController.calculate(right.getCurrentPosition(), targetHeight)) + kg + add);
//            left.set((leftController.calculate(left.getCurrentPosition(), targetHeight)) + kg + add);
//        }


        // && Math.abs(right.getCurrentPosition() - targetHeight) > 25
        if (right.getCurrentPosition() - targetHeight > 0) { // use profile
            if (targetHeight == 0) {

            } else if (right.getCurrentPosition() < jamHeight) {
                Log.d("asd", "under " + right.getCurrentPosition());
                right.set(jamPIDRight.calculate(right.getCurrentPosition(), targetHeight) + jkg);
                left.set(jamPIDLeft.calculate(left.getCurrentPosition(), targetHeight) + jkg);
            } else if (profileDown) {
                right.set((downRightProfiledController.calculate(right.getCurrentPosition(), targetHeight)) + dkg);
                left.set((downLeftProfiledController.calculate(left.getCurrentPosition(), targetHeight)) + dkg);
            } else {
                right.set((downRightController.calculate(right.getCurrentPosition(), targetHeight)) + dkg);
                left.set((downLeftController.calculate(left.getCurrentPosition(), targetHeight)) + dkg);
            }
        } else {
            if (profileUp) {
                right.set((upRightProfiledController.calculate(right.getCurrentPosition(), targetHeight)) + ukg);
                left.set((upLeftProfiledController.calculate(left.getCurrentPosition(), targetHeight)) + ukg);
            } else {
                right.set((upRightController.calculate(right.getCurrentPosition(), targetHeight)) + ukg);
                left.set((upLeftController.calculate(left.getCurrentPosition(), targetHeight)) + ukg);
            }
        }
//        }
    }
}
