package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
public class LiftSys extends SubsystemBase {
    private final MotorEx left,right;

    public static double NONE = 0;
    public static double LOW = 200;
    public static double MID = 400;
    public static double HIGH = 600;

    public static double ki = 0;
    public static double kp = 0.02;
    public static double kd = 0;
    public static double kf = 0.14;

    public static double maxVel = 500;
    public static double maxAccel = 500;
    public static double targetHeight;
    public static int posThreshold = 10;
    public static int velThreshold = 10;
    private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(maxVel, maxAccel);
    private final ProfiledPIDController Rcontroller = new ProfiledPIDController(kp,ki,kd,constraints);
    private final ProfiledPIDController Lcontroller = new ProfiledPIDController(kp,ki,kd,constraints);

    private final TouchSensor limitSwitch;

    private ElapsedTime voltageTimer;
    private VoltageSensor voltageSensor;
    private double voltage;

    public LiftSys(MotorEx left, MotorEx right, TouchSensor sensor, HardwareMap.DeviceMapping<VoltageSensor> voltageSensor) {
        this.left =left;
        this.right = right;
        this.left.resetEncoder();
        this.right.resetEncoder();
        this.limitSwitch = sensor;
        Rcontroller.setPID(kp,ki,kd);
        Rcontroller.setTolerance(posThreshold,velThreshold);
        Lcontroller.setPID(kp,ki,kd);
        Lcontroller.setTolerance(posThreshold,velThreshold);
        this.voltageTimer = new ElapsedTime();
        this.voltageTimer.reset();
        this.voltageSensor = voltageSensor.iterator().next();
        this.voltage = this.voltageSensor.getVoltage();
        targetHeight = 0;
    }

    public void setHeight(double height) {
        targetHeight = height;
    }
//    public Command goTo(double height) {
//        return new InstantCommand(() -> setHeight(height))
//                .andThen(new WaitUntilCommand(this::atTarget));
//    }
//    public boolean atTarget() {
//        return right.getCurrentPosition() < targetHeight + threshold && right.getCurrentPosition() > targetHeight - threshold
//                || left.getCurrentPosition() < targetHeight + threshold && left.getCurrentPosition() > targetHeight - threshold;
//    }
    public double getVoltage(){return voltage;}
    public double getPosErrorL(){return Lcontroller.getPositionError();}
    public double getProfilePowerL(){return (Lcontroller.calculate(right.getCurrentPosition(), targetHeight)) + kf;}
    public double getPowerL(){return left.get();}

    @Override
    public void periodic() {
        if(limitSwitch.isPressed()) {
            left.resetEncoder();
            right.resetEncoder();
        }
        if (voltageTimer.seconds() > 5) {
            voltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }
//        if(left.getCurrentPosition() == 0 && getTargetHeight() == 0) {
//            // do nothing
//        } else {
            right.set((Rcontroller.calculate(right.getCurrentPosition(), targetHeight)) + kf);
            left.set((Lcontroller.calculate(left.getCurrentPosition(), targetHeight)) + kf);
//        }
    }
}
