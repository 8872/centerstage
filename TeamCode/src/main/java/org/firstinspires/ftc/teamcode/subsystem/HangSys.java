package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class HangSys extends SubsystemBase {
    private MotorEx hang;
    private boolean hasHung;
    public static double releaseHeight = 200;
    public static double holdHeight = 150;
    public final int STAGE_CONSTANT;
    public HangSys(MotorEx hang) {
        this.hang = hang;
        STAGE_CONSTANT = (int) (hang.getCPR());

    }
    public Command setRelease() {
        if (!hasHung) {
            hasHung = true;
            return new InstantCommand(() -> moveToPosition(3*STAGE_CONSTANT + 8));
        } else {
            return new InstantCommand();
        }
    }
    public Command setHang() {
        if (hasHung) {
            return new InstantCommand(()-> moveToPosition(2*STAGE_CONSTANT + 4));
        }
        return new InstantCommand();
    }

    public void moveToPosition(int position) {
        hang.setTargetPosition(position);
        hang.setRunMode(MotorEx.RunMode.PositionControl);
        hang.set(position);
    }

    public void resetPositionCounter() {
        hang.resetEncoder();
    }
}
