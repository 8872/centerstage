package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class NormalRROpMode extends OpMode {
    SampleMecanumDrive drive;
    TrajectorySequence a, b;

    public static boolean runA;
    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        a = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .forward(20)
                .build();
        b = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .turn(Math.toRadians(90)).build();
    }

    @Override
    public void loop() {
        if (runA) {
            drive.followTrajectorySequence(a);
        } else {
            drive.followTrajectorySequence(b);
        }
//        drive.update();
    }
}
