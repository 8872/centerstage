package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ProfiledPIDCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;

import java.util.function.DoubleSupplier;

@Config
public class LiftSys extends SubsystemBase {
    public enum Height {
        NONE(0),
        LOW(400),
        MEDIUM(800),
        HIGH(1600);
        public final int pos;
        Height(int pos) {
            this.pos = pos;
        }
    }

    public static int none = 0;
    public static int low = -223;
    public static int medium = -446;
    public static int high = -670;
    public static int max = -670;
    private static Height currentGoal = Height.NONE;
    public static double kP = 0.00;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kG = 0.0;
    public static double maxVelocity = 0;
    public static double maxAcceleration = 0;
    private final ProfiledPIDController controller = new ProfiledPIDController(kP, kI, kD,
            new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    public static int threshold = 0;
    public static double slow = 0;
    private final DoubleSupplier doubleSupplier;
    private int currentTarget;
    public MotorEx lil;
    public MotorEx lir;

    public LiftSys(MotorEx lil, MotorEx lir, DoubleSupplier doubleSupplier) {
        this.lil = lil;
        this.lir = lir;
        this.doubleSupplier = doubleSupplier;
    }

    public void setHeight(Height junction){
        currentGoal = junction;
        switch (junction) {
            case NONE:
                currentTarget = none;
                controller.setGoal(none);
                break;
            case LOW:
                currentTarget = low;
                controller.setGoal(low);
                break;
            case MEDIUM:
                currentTarget = medium;
                controller.setGoal(medium);
                break;
            case HIGH:
                currentTarget = high;
                controller.setGoal(high);
                break;
        }
    }

    public void setHigh() {
        setHeight(Height.HIGH);
    }
    public void setNone() {
        setHeight(Height.NONE);
    }
    public void setLow() {
        setHeight(Height.LOW);
    }
    public void setMedium() {
        setHeight(Height.MEDIUM);
    }

    public boolean atTarget(){
        switch(currentGoal){
            case NONE:
                return lir.getCurrentPosition()< none+ threshold && lir.getCurrentPosition()>none- threshold;
            case LOW:
                return lir.getCurrentPosition()< low+ threshold && lir.getCurrentPosition()>low- threshold;
            case MEDIUM:
                return lir.getCurrentPosition()< medium+ threshold && lir.getCurrentPosition()>medium- threshold;
            case HIGH:
                return lir.getCurrentPosition()< high+ threshold && lir.getCurrentPosition()>high- threshold;

        }
        return false;
    }

    public Height getCurrentJunction() {
        return currentGoal;
    }

    public int getCurrentTarget() {
        return currentTarget;
    }

    public static boolean isDown() {
        return currentGoal == Height.NONE;
    }

    @Override
    public void periodic() {
        if(doubleSupplier.getAsDouble() != 0) {
            lir.set(doubleSupplier.getAsDouble()/slow);
            lil.set(doubleSupplier.getAsDouble()/slow);
            controller.setGoal(lir.getCurrentPosition());
        } else {
            lil.set(controller.calculate(lir.getCurrentPosition()) + kG);
            lir.set(controller.calculate(lir.getCurrentPosition()) + kG);
        }
    }
}
