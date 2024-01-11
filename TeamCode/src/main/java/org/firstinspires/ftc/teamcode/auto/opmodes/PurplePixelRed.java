package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.pipelines.RedZoneDetectionProcessor;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives back and forth in a straight line indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin moving forward and
 * backward. You should observe the target position (green) and your pose estimate (blue) and adjust
 * your follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 *
 * This opmode is designed as a convenient, coarse tuning for the follower PID coefficients. It
 * is recommended that you use the FollowerPIDTuner opmode for further fine tuning.
 */

@Config
@Autonomous(group = "drive")
public class PurplePixelRed extends LinearOpMode {

    public static double x1 = 28;
    public static double y1 = 5.5;
    public static double x2 = 27.5;
    public static double y2 = 0;
    public static double x3 = 28;
    public static double y3 = -5.5;

    int counter = 0;

   RedZoneDetectionProcessor processor;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        MotorEx intakeMotor = new MotorEx(hardwareMap, "intakeMotor");
        SimpleServo stackServo = new SimpleServo(hardwareMap, "stackServo", 0, 255);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);

        IntakeSubsystem intakeSys = new IntakeSubsystem(intakeMotor, stackServo, () -> gamepadEx2.gamepad.right_trigger, () -> gamepadEx2.gamepad.left_trigger);

        processor = new RedZoneDetectionProcessor();
        new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .build();

        FtcDashboard.getInstance().startCameraStream(processor, 0);


        Trajectory pushPurp1 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(x1, y1), 0.35,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory pushPurp2 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(x2, y2), 0,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory pushPurp3 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(x3, y3), -0.35,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if(counter == 0){
                CommandScheduler.getInstance().schedule(intakeSys.stackUp());
                sleep(2000);
                counter++;
            }if(counter == 1){
                int zone = processor.getZone();
                switch(zone){
                    case 1:
                        drive.followTrajectory(pushPurp1);
                        break;
                    case 2:
                        drive.followTrajectory(pushPurp2);
                        break;
                    default:
                        drive.followTrajectory(pushPurp3);
                }
                counter++;
            }
            telemetry.addData("zone", processor.getZone());
            telemetry.update();



        }
    }
}