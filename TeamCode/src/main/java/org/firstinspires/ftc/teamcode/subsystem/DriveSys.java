package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSys extends SubsystemBase {
    /**
     * default drive system
     */
    private final MecanumDrive drive;
    public static double slowMode = 0.5;
    public DriveSys(MotorEx leftBack, MotorEx leftFront, MotorEx rightBack, MotorEx rightFront) {
        drive = new MecanumDrive(true,leftFront, rightFront, leftBack, rightBack);
        //autoDrive = new org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive(hardwareMap, new Pose2d(0,0,0));
    }
    public void driveFieldCentric(double forward, double strafe, double rotation, double heading) {
        drive.driveFieldCentric(forward, strafe, rotation, heading);
    }
    public void driveRobotCentric(double forward, double strafe, double rotation) {
        drive.driveRobotCentric(forward, strafe, rotation);
    }
    public void driveRobotCentricSlowMode(double forward, double strafe, double rotation) {
        drive.driveRobotCentric(forward * slowMode, strafe * slowMode, rotation * slowMode);
    }



}
