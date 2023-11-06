package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.function.DoubleSupplier;

public class DriveSys extends SubsystemBase {
    private final MecanumDrive drive;
    private final IMU imu;

    public DriveSys(MotorEx fL, MotorEx fR, MotorEx bL, MotorEx bR, IMU imu) {
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
