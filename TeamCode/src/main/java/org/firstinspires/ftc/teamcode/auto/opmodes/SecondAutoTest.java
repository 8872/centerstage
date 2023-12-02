//package org.firstinspires.ftc.teamcode.auto.opmodes;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
//import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
//
///*
// * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
// * classes). The robot drives back and forth in a straight line indefinitely. Utilization of the
// * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
// * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
// * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
// * you've successfully connected, start the program, and your robot will begin moving forward and
// * backward. You should observe the target position (green) and your pose estimate (blue) and adjust
// * your follower PID coefficients such that you follow the target position as accurately as possible.
// * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
// * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
// * These coefficients can be tuned live in dashboard.
// *
// * This opmode is designed as a convenient, coarse tuning for the follower PID coefficients. It
// * is recommended that you use the FollowerPIDTuner opmode for further fine tuning.
// */
//@Autonomous(group = "drive")
//public class SecondAutoTest extends LinearOpMode {
//
//    public static double x1 = 28;
//    public static double y1 = 4.8;
//    public static double x2 = 28;
//    public static double y2 = 0;
//    public static double x3 = 28;
//    public static double y3 = -4.8;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//
//        Trajectory pushPurp1 = drive.trajectoryBuilder(new Pose2d())
//                .splineTo(new Vector2d(x1, y1), 0.35,
//                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();
//        Trajectory pushPurp2 = drive.trajectoryBuilder(new Pose2d())
//                .splineTo(new Vector2d(x1, y1), 0,
//                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();
//        Trajectory pushPurp3 = drive.trajectoryBuilder(new Pose2d())
//                .splineTo(new Vector2d(x1, y1), -0.35,
//                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();
//
//        Trajectory trajectoryBackward = drive.trajectoryBuilder(pushPurp.end())
//                .back(DISTANCE)
//                .build();
//
//        waitForStart();
//
//        while (opModeIsActive() && !isStopRequested()) {
//            drive.followTrajectory(pushPurp);
//            drive.followTrajectory(trajectoryBackward);
//        }
//    }
//}