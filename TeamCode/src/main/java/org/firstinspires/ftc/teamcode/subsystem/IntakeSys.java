package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Precision;

import java.util.function.DoubleSupplier;
@Config
public class IntakeSys extends SubsystemBase {
    public static boolean sensors = true;
    public enum IntakeState {
        ONE,
        TWO,
        NONE,
        TWO_AND_ONE,
        NONE_AND_ONE
    }
    public IntakeState state = IntakeState.NONE;
    private final MotorEx intake;
    private final SimpleServo stack;
    public static double intakeInPower = 0.5;
    public static double intakeOutPower = 0;
    public static double intakeServoLowPosition = 0;
    public static double intakeServoHighPosition = 0.5;
    private final double[] coefficients;
    private final Rev2mDistanceSensor distanceSensor, breakBeamSensor;
    public IntakeSys(SimpleServo stack, MotorEx intake, Rev2mDistanceSensor distanceSensor, Rev2mDistanceSensor breakBeamSensor) {
         this.stack = stack;
         this.intake = intake;
        this.distanceSensor = distanceSensor;
        this.breakBeamSensor = breakBeamSensor;
         coefficients = Precision.calculateSlopeAndIntercept(0,intakeServoHighPosition,1,intakeServoLowPosition);
    }
    public double getCurrent() {
        return intake.motorEx.getCurrent(CurrentUnit.MILLIAMPS);
    }
    public Command intake(DoubleSupplier fpower, DoubleSupplier rpower) {
        return new RunCommand(() -> {
            if(sensors) {
                if(state == IntakeState.TWO_AND_ONE) {
                    stack.setPosition(intakeServoHighPosition);
                    intake.set(-1);
                } else if (state == IntakeState.TWO) {
                    stack.setPosition(intakeServoHighPosition);
                    intake.set(0);
                } else {
                    if (fpower.getAsDouble() != 0) {
                        intake.set(intakeInPower);
                        stack.setPosition((fpower.getAsDouble()* coefficients[0])+ coefficients[1]);
                    }
                }
            } else {
                if (fpower.getAsDouble() != 0) {
                    intake.set(intakeInPower);
                    stack.setPosition((fpower.getAsDouble()* coefficients[0])+ coefficients[1]);
                }
            }
            if (rpower.getAsDouble() != 0) {
                intake.set(intakeOutPower);
                stack.setPosition((rpower.getAsDouble()* coefficients[0])+ coefficients[1]);
            } else {
                intake.set(0);
                stack.setPosition(intakeServoHighPosition);
            }
        }, this);
    }
    public Command runIntake(double power) {
        return new RunCommand(() -> {
            intake.set(intakeInPower);
        }, this);
    }
    public Command setStack(double position) {
        return new InstantCommand(() -> {
            stack.setPosition(position);
        }, this);
    }

    public IntakeState getState() {
        if(distanceSensor.getDistance(DistanceUnit.CM) < 6 && breakBeamSensor.getDistance(DistanceUnit.INCH) < 1.5) {
            return IntakeState.TWO_AND_ONE;
        } else if(distanceSensor.getDistance(DistanceUnit.CM) < 10) {
            if(distanceSensor.getDistance(DistanceUnit.CM) < 6) {
                return IntakeState.TWO;
            } else {
                return IntakeState.ONE;
            }
        } else {
            return IntakeState.NONE;
        }
    }
    @Override
    public void periodic() {
        state = getState();
    }
}
