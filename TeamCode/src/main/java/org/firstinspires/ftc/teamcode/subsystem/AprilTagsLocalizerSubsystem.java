package org.firstinspires.ftc.teamcode.subsystem;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.*;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.ArrayList;
import java.util.List;

public class AprilTagsLocalizerSubsystem extends SubsystemBase implements Localizer {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Pose2d poseEstimate;
    public AprilTagsLocalizerSubsystem(CameraName camera) {
        aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(camera);
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
        visionPortal.setProcessorEnabled(aprilTag, true);
    }

    private Pose2d getPoseFromDetection(AprilTagDetection detection) {
        AprilTagMetadata reference = AprilTagGameDatabase.getCenterStageTagLibrary().lookupTag(detection.id);

        if (reference != null) {
            AprilTagPoseFtc detectedPose = detection.ftcPose;

            double range = detectedPose.range;
            double bearing = detectedPose.bearing;
            double yaw = detectedPose.yaw;

            double theta = (bearing + (Math.PI / 2)) + yaw;

            double x = reference.fieldPosition.get(0) + (range * Math.cos(theta));
            double y = reference.fieldPosition.get(1) + (range * Math.sin(theta));

            double heading = Math.toRadians(detection.id > 6? 0:180) + yaw;


            return new Pose2d(x, y, heading);
        }
        return null;
    }
    @NotNull
    @Override
    public Pose2d getPoseEstimate() {return poseEstimate;}
    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        poseEstimate = pose2d;
    }
    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }

    @Override
    public void update() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        List<Pose2d> possiblePoses = new ArrayList<>();
        for (AprilTagDetection detection : detections) {
            Pose2d pose = getPoseFromDetection(detection);
            if (pose != null) {
                possiblePoses.add(pose);
            }
        }
        if (!possiblePoses.isEmpty()) {
            double x_sum = 0;
            double y_sum = 0;
            double heading_sum = 0;
            for (Pose2d possiblePose : possiblePoses) {
                x_sum += possiblePose.getX();
                y_sum += possiblePose.getY();
                heading_sum += possiblePose.getHeading();
            }
            setPoseEstimate(new Pose2d(x_sum / possiblePoses.size(), y_sum / possiblePoses.size(), heading_sum / possiblePoses.size()));

        }
    }
}
