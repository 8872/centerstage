package org.firstinspires.ftc.teamcode.auto.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class PushPreloadTrajec extends CommandBase {
    private final SampleMecanumDrive drive;
    private int zone;
    public static double x1 = 28;
    public static double y1 = -10;
    public static double x2 = 28;
    public static double y2 = 0;
    public static double x3 = 28;
    public static double y3 = 10;


    public PushPreloadTrajec(SampleMecanumDrive drive, int zone) {
        this.drive = drive;
        this.zone = zone;
    }

    @Override
    public void initialize() {
        switch(zone){
            case 1:
                drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                        .splineTo(new Vector2d(x1, y1), Math.toRadians(120))
                        .build());
                break;
            case 2:
                drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                        .splineTo(new Vector2d(x2, y2), Math.toRadians(90))
                        .build());
                break;
            case 3:
                drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                        .splineTo(new Vector2d(x3, y3), Math.toRadians(60))
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

