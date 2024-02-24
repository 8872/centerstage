package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.Precision;

import java.util.function.DoubleSupplier;

@Config
public class IntakeSys extends SubsystemBase {
    public static boolean algorithm = true;
    public static int spikeCount = 0;
    private boolean lastActive = false;
    private long spikeStartTime = 0;
    private MotorEx intake;
    private SimpleServo stack;
    private SimpleServo stack2;

    public static double intakeInPower = 0.7;

    public static double intakeOutPower = 0.7;

    public static double intakeServoLowPosition = 0.5;
    public static double intakeServoHighPosition = 0.2;

    public static double intakeServo2LowPosition = 0.55;
    public static double intakeServo2HighPosition = 0.31;
    private final double[] coefficients;
//    private final double[] coefficients2;

    public IntakeSys(SimpleServo stack, SimpleServo stack2, MotorEx intake) {
        this.stack = stack;
        this.stack2 = stack2;
        this.intake = intake;
        coefficients = Precision.calculateSlopeAndIntercept(0, intakeServoHighPosition, 1, intakeServoLowPosition);
    }

    public double getCurrent() {
        return intake.motorEx.getCurrent(CurrentUnit.MILLIAMPS);
    }

    public Command intake(DoubleSupplier fpower, DoubleSupplier rpower) {
        return new RunCommand(() -> {
            if (fpower.getAsDouble() != 0) {
                intake.set(intakeInPower);
                stack.setPosition((fpower.getAsDouble() * coefficients[0]) + coefficients[1]);
                stack2.setPosition((fpower.getAsDouble() * coefficients[0]) + coefficients[1]);
            } else if (rpower.getAsDouble() != 0) {
                intake.set(-intakeOutPower);
                stack.setPosition((rpower.getAsDouble() * coefficients[0]) + coefficients[1]);
                stack2.setPosition((rpower.getAsDouble() * coefficients[0]) + coefficients[1]);
            } else {
                intake.set(0);
                stack.setPosition(intakeServoHighPosition);
                stack2.setPosition(intakeServoHighPosition);
            }
        }, this);
    }

    public Command runIntake(double power) {
        return new RunCommand(() -> {
            intake.set(intakeInPower);
        }, this);
    }

    public Command setStack1(double position) {
        return new InstantCommand(() -> {
            stack.setPosition(position);
        }, this);
    }

    public Command setStack2(double position) {
        return new InstantCommand(() -> {
            stack.setPosition(position);
        }, this);
    }

    public double getIntakePower(){
        return intake.get();
    }

    @Override
    public void periodic() {

//        if (algorithm) {
//            double current = getCurrent();
//            double forwardPower = intake.get();
//            if (current > 3000 && forwardPower > 0) {
//                if (!spikeActive) {
//                    spikeActive = true;
//                    spikeStartTime = System.currentTimeMillis();
//                }
//            } else {
//                if (spikeActive) {
//                    long spikeDuration = System.currentTimeMillis() - spikeStartTime;
//                    if (spikeDuration >= 100) {
//                        spikeCount++;
//                    }
//                    if (spikeDuration >= 700) {
//                        intake.set(-1);
//                        long reverseStartTime = System.currentTimeMillis();
//                        while (System.currentTimeMillis() - reverseStartTime < 2000) {
//                        }
//                        intake.set(0);
//                    }
//                    spikeActive = false;
//                }
//            }
//        }
    }
}
