package org.firstinspires.ftc.teamcode.subsystem;

import android.util.Log;
import com.arcrobotics.ftclib.command.*;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import java.util.function.DoubleSupplier;

public class HangSubsystem extends SubsystemBase {
    private final MotorEx hang;
    private final DoubleSupplier manual;
    private boolean inDelay = false;

    public HangSubsystem(MotorEx hang, DoubleSupplier manual) {
        this.hang = hang;
        this.manual = manual;
    }

    public Command delayed(double power, int delay) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    hang.set(power);
                    inDelay = true;
                }),
                new WaitCommand(delay),
                new InstantCommand(() -> {
                    hang.set(0);
                    inDelay = false;
                })
        );
    }

    @Override
    public void periodic() {
        if (!inDelay) {
            hang.set(manual.getAsDouble());
        }
    }
}
