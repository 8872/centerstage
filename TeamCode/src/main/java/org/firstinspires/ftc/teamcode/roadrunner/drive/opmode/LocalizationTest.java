package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.subsystem.AprilTagsLocalizerSubsystem;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        AprilTagsLocalizerSubsystem aprilTagsLocalizerSubsystem = new AprilTagsLocalizerSubsystem(hardwareMap.get(WebcamName.class, "webcam"));
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();

        int leftEncStart = drive.localizer.leftEncoder.getCurrentPosition();
        int rightEncStart = drive.localizer.rightEncoder.getCurrentPosition();
        int frontEncStart = drive.localizer.frontEncoder.getCurrentPosition();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();
            aprilTagsLocalizerSubsystem.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("left encoder", drive.localizer.leftEncoder.getCurrentPosition()-leftEncStart);
            telemetry.addData("right encoder", drive.localizer.rightEncoder.getCurrentPosition()-rightEncStart);
            telemetry.addData("front encoder", drive.localizer.frontEncoder.getCurrentPosition()-frontEncStart);
            telemetry.addData("Pose X", aprilTagsLocalizerSubsystem.getPoseEstimate().getX());
            telemetry.addData("Pose Y", aprilTagsLocalizerSubsystem.getPoseEstimate().getY());
            telemetry.addData("Pose Heading", aprilTagsLocalizerSubsystem.getPoseEstimate().getHeading());
            telemetry.update();
        }
    }
}
