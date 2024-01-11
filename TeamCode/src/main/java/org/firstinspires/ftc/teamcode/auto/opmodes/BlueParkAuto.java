package org.firstinspires.ftc.teamcode.auto.opmodes;

import android.util.Size;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.commands.PushPreloadTrajec;
import org.firstinspires.ftc.teamcode.opmode.DriveBaseOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.commands.DelayedCommand;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous
public class BlueParkAuto extends TestBaseOpMode {

    SampleMecanumDrive rrDrive;
    private int classZone;
    private int counter = 0;

    //set pos to (0,0,Math.toRadian(90))
    //init
    //lower intake
    //command to retrieve zone detection from StandardTrackingWheelLocalizer
    //parallel call PushPreloadTrajec (pass zone) and raise intake
    //linetosplineheading to center of tile
    //parallel: line to in front of the board and commandseq to raise lift when past barrier
    //strafe based on zone
    //release and back up one inch

//    BlueZoneDetectionProcessor processor;

    @Override
    public void initialize() {
        super.initialize();

        rrDrive = new SampleMecanumDrive(hardwareMap);
        rrDrive.setPoseEstimate(new Pose2d(0, 0, 0));

    }

    @Override
    public void run(){
        sleep(5000);
        TrajectorySequence ts = rrDrive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .forward(4)
                .turn(Math.toRadians(90)) // Turns 45 degrees counter-clockwise
                .forward(48)
                .build();

        rrDrive.followTrajectorySequence(ts);

    }

}
