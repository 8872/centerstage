package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class IntakeSys extends SubsystemBase {
    MotorEx intakeMotor;
    ServoEx stackServoL;
    ServoEx stackServoR;

    public static double currentThreshold = 0.0;
    public enum StackHeight {
        LOW(0.1),
        MEDIUM(0.2),
        HIGH(0.3);
        public final double pos;
        StackHeight(double pos) {
            this.pos = pos;
        }
    }

    public IntakeSys(MotorEx intakeMotor, ServoEx stackServoL, ServoEx stackServoR) {
        this.intakeMotor = intakeMotor;
        this.stackServoL = stackServoL;
        this.stackServoR = stackServoR;
    }

    public void intake(){
        if(intakeMotor.motorEx.getCurrent(CurrentUnit.AMPS) > currentThreshold){
            intakeMotor.set(-1);
        } else {
            intakeMotor.set(1);
        }
    }

    public void stop() {
        intakeMotor.set(0);
    }
    public void outtake() {
        intakeMotor.set(-1);
    }
    public void stackHeight(StackHeight height) {
        stackServoL.setPosition(height.pos);
        stackServoR.setPosition(height.pos);
    }
}
