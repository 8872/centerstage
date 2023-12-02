package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.function.DoubleSupplier;

@Config
public class IntakeSubsystem extends SubsystemBase {
    public static double LOW = 0.9;
    public static double HIGH = 0.4;
    public static double IN = 0.7;
    public static double OUT = -1;
    public static double STOP = 0;


    private final DoubleSupplier fSPower;
    private final DoubleSupplier rSPower;
    private final MotorEx intake;
    private final ServoEx stack;

    public IntakeSubsystem(MotorEx intake, ServoEx stack, DoubleSupplier fSPower, DoubleSupplier rSPower) {
        this.intake = intake;
        intake.motorEx.setCurrentAlert(9.2, CurrentUnit.AMPS);
        this.stack = stack;
        this.fSPower = fSPower;
        this.rSPower = rSPower;
    }

    public Command in() {
        return new InstantCommand(() -> intake.set(IN));
    }

    public Command out() {
        return new InstantCommand(() -> intake.set(OUT));
    }

    public Command stackDown(){
        return new InstantCommand(() -> stack.setPosition(0.9));
    }

    public Command stackUp(){
        return new InstantCommand(() -> stack.setPosition(0.42));
    }

    public Command stop() {
        return new InstantCommand(() -> intake.set(STOP));
    }

    public Command setHeight(double height) {
        return new InstantCommand(() -> stack.setPosition(height));
    }

    public boolean getCurrent() {return intake.motorEx.isOverCurrent();}

    @Override
    public void periodic() {
        if (getCurrent()) {
            intake.set(0);
            stack.setPosition(HIGH);
        } else if(fSPower.getAsDouble()!=0) {
            intake.set(IN);
            stack.setPosition((fSPower.getAsDouble()*0.5)+0.4);
        } else if (rSPower.getAsDouble() !=0) {
            intake.set(OUT);
            stack.setPosition((rSPower.getAsDouble()*0.5)+0.4);
        } else {
            stack.setPosition(HIGH);
            intake.set(0);
        }
    }
}
