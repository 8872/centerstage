package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.function.DoubleSupplier;

public class DriveSubsystem extends SubsystemBase {
    private final MecanumDrive drive;
    private final IMU imu;
    private double slowFactor = 1;

    public DriveSubsystem(MotorEx fL, MotorEx fR, MotorEx bL, MotorEx bR, IMU imu) {
        drive = new MecanumDrive(fL, fR, bL, bR);
        this.imu = imu;
    }

//    public Command drive(DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier turn) {
//        return drive(strafe, forward, turn, 1);
//    }

    public Command drive(DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier turn) {
        return new RunCommand(
                () -> drive.driveRobotCentric(strafe.getAsDouble() * slowFactor, forward.getAsDouble()* slowFactor,
                        turn.getAsDouble()* slowFactor),
                this
        );
    }

    public Command startSlow(){
        return new InstantCommand(() -> slowFactor = 0.5);
    }
    public Command stopSlow(){
        return new InstantCommand(() -> slowFactor = 1);
    }

}
