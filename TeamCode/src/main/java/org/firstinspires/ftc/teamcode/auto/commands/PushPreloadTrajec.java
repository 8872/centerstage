package org.firstinspires.ftc.teamcode.auto.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class PushPreloadTrajec extends CommandBase {

    private final SampleMecanumDrive drive;
    private int zone;
    public static double x1 = 28;
    public static double y1 = 4.8;
    public static double x2 = 28;
    public static double y2 = 0;
    public static double x3 = 28;
    public static double y3 = -4.8;


    public PushPreloadTrajec(SampleMecanumDrive drive, int zone) {
        this.drive = drive;
        this.zone = zone;
    }

    @Override
    public void initialize() {
        switch(zone){
            case 1:
                drive.followTrajectoryAsync(drive.trajectoryBuilder(new Pose2d(0, -0.00, Math.toRadians(0.00)))
                        .splineTo(new Vector2d(x1, y1), 0.35,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build());
                break;
            case 2:
                drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                        .splineTo(new Vector2d(x2, y2), 0,
                                SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build());
                break;
            case 3:
                drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                        .splineTo(new Vector2d(x3, y3), 0.35,
                                SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build());
                 break;
            default:
                drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                        .splineTo(new Vector2d(x3, y3), -0.35,
                                SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build());
                break;
        }
    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public boolean isFinished() {
        return !drive.isBusy();
    }
}

