package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
@Config
public class IntakeSys extends SubsystemBase {
    MotorEx intakeMotor;
    SimpleServo stackServo;
    public static double currentThreshold = 1000;
    public enum StackHeight {
        LOW(0.05),
        MEDIUM(0.2),
        HIGH(0.4);
        public final double pos;
        StackHeight(double pos) {
            this.pos = pos;
        }
    }

    public enum IntakeDirection {
        IN(0.7),
        OUT(-1.0),
        STOP(0.0);
        public final double power;
        IntakeDirection(double power) {
            this.power = power;
        }
    }
    public static double power = 0.8;
    public StackHeight stackHeight;
    public IntakeDirection intakeDirection;
    public IntakeSys(MotorEx intakeMotor, SimpleServo stackServoL) {
        this.intakeMotor = intakeMotor;
        this.stackServo = stackServoL;
        stackHeight = StackHeight.LOW;
    }
    public void intake(IntakeDirection direction) {
        if(intakeMotor.motorEx.getCurrent(CurrentUnit.AMPS) > currentThreshold){
            intakeMotor.set(IntakeDirection.OUT.power);
            intakeDirection = IntakeDirection.OUT;
        } else {
            intakeMotor.set(direction.power);
            intakeDirection = direction;
        }
    }
    public void stackHeight(StackHeight height) {
        stackServo.setPosition(height.pos);
        stackHeight = height;
    }
    public StackHeight getStackHeight() {
        return stackHeight;
    }
    public double getStackHeightPos() {
        return stackServo.getPosition();
    }
    public IntakeDirection getIntake() {
        return intakeDirection;
    }
}
