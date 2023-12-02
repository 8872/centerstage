//package org.firstinspires.ftc.teamcode.auto.commands;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.arcrobotics.ftclib.command.CommandBase;
//import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
//import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
//
//public class GoToCenter extends CommandBase {
//
//    private final SampleMecanumDrive drive;
//    public static double x = 20;
//    public static double y = 0;
//
//
//    public GoToCenter(SampleMecanumDrive drive, int zone) {
//        this.drive = drive;
//    }
//
//    @Override
//    public void initialize() {
//
//        drive.followTrajectoryAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                .back(2)
//                .lineToSplineHeading(new Pose2d(x, y, Math.toRadians(90)),
//                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build());
//    }
//
//    @Override
//    public void execute() {
//        drive.update();
//    }
//
//    @Override
//    public boolean isFinished() {
//        return !drive.isBusy();
//    }
//}
//
