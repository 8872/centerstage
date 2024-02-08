package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import java.util.function.DoubleSupplier;

public class DriveSys extends SubsystemBase {
    private final MecanumDrive drive;
    public DriveSys(MotorEx fL, MotorEx fR, MotorEx bL, MotorEx bR) {
        drive = new MecanumDrive(fL, fR, bL, bR);
    }
    public Command drive(DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier turn, double slowFactor) {
        return new RunCommand(
                () -> drive.driveRobotCentric(-strafe.getAsDouble() * slowFactor, -forward.getAsDouble()* slowFactor,
                        turn.getAsDouble()* slowFactor),
                this
        );
    }


}
