package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.*;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import java.util.function.DoubleSupplier;

public class DriveSubsystem extends SubsystemBase {
    private final MecanumDrive drive;
    private final RevIMU imu;

    public DriveSubsystem(MotorEx fL, MotorEx fR, MotorEx bL, MotorEx bR, RevIMU imu) {
        drive = new MecanumDrive(fL, fR, bL, bR);
        this.imu = imu;
    }

    public Command drive(DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier turn) {
        return drive(strafe, forward, turn, 1);
    }

    public Command drive(DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier turn, double slowFactor) {
        return new RunCommand(
                () -> drive.driveRobotCentric(strafe.getAsDouble() / slowFactor, forward.getAsDouble(),
                        turn.getAsDouble()),
                this
        );
    }


}