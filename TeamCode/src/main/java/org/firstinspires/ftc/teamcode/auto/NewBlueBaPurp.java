package org.firstinspires.ftc.teamcode.auto;

import android.util.Size;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.CV.HSVDetectionPipeline;
import org.firstinspires.ftc.teamcode.auto.CV.Side;
import org.firstinspires.ftc.teamcode.auto.util.AutoBaseOpmode;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.util.HighestSampleFilter;
import org.firstinspires.ftc.teamcode.util.commands.DelayedCommand;
import org.firstinspires.ftc.teamcode.util.wpilib.LinearFilter;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name="New Blue Ba Purple", group = "Auto")
public class NewBlueBaPurp extends AutoBaseOpmode {

    private HSVDetectionPipeline processor;
    private VisionPortal portal;

    public static int sampleSize = 5;
    private final HighestSampleFilter filter = new HighestSampleFilter(sampleSize);

    public enum State{
        WAIT_FOR_START,
        MOVE_TO_PROP,
        EJECT_AND_PARK,
        DEPOSIT,
        DROP,
        WAIT_FOR_FINISH,
        FINISHED
    }
    public static State currentState = State.WAIT_FOR_START;

    public static boolean red = false;
    public static int zone = 2;

    public static double x1 = -46;
    public static double y1 = 37;
    public static double x2 = -35;
    public static double y2 = 35.5;
    public static double x3 = -29;
    public static double y3 = 38;
    public static double angle2 = 80;
    public static double angle3 = 60;

    public static int taps = 10;
    private final LinearFilter lowPass = LinearFilter.movingAverage(taps);

    private int pixelCount = 0;

    public static double FIRST = 123;
    public static double SECOND = 105;
    public static double tolerance = 4;

    private ElapsedTime depositWaitTimer;

    public static double backdropX = -78;
    public static double backdropY1 = -50;
    public static double backdropY2 = -44;
    public static double backdropY3 = -38;

    private double BLDistance;

    @Override
    public void init(){
        super.init();
        depositWaitTimer = new ElapsedTime();
        currentState = State.WAIT_FOR_START;
        drive.setPoseEstimate(new Pose2d(-41.75, -63.00, Math.toRadians(90.00)));

        processor = new HSVDetectionPipeline(Side.BLUE_CLOSE);
        portal = new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(1920, 1080))
                .setAutoStopLiveView(true)
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .build();

        telemetry.addData("Initialization done", true);
    }

    @Override
    public void init_loop(){
        telemetry.addData("zone", processor.getZone());
        telemetry.update();
    }

    @Override
    public void start(){
        depositWaitTimer.reset();
        schedule(armSubsystem.intake());
        schedule(intakeSubsystem.setStack1(IntakeSubsystem.servoLowPosition-0.05));
        schedule(intakeSubsystem.setStack2(IntakeSubsystem.servo2LowPosition-0.05));
        currentState = State.MOVE_TO_PROP;
    }

    @Override
    public void loop() {
        telemetry.addData("zone", processor.getZone());
        super.loop();
        telemetry.addData("raw bl data", localizerSubsystem.getBl());
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        drive.update();
        liftSubsystem.periodic();
        BLDistance = filter.calculate(lowPass.calculate(localizerSubsystem.getBl()));
        if(currentState == State.MOVE_TO_PROP && depositWaitTimer.milliseconds() > 2500){
            zone = processor.getZone();
            portal.close();
            schedule(intakeSubsystem.setStack1(IntakeSubsystem.servoHighPosition));
            schedule(intakeSubsystem.setStack2(IntakeSubsystem.servo2HighPosition));

            switch(zone) {
                case 1:
                    drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(-41.75, -63.00, Math.toRadians(90.00)))
                            .lineToSplineHeading(new Pose2d(x1, -y1, Math.toRadians(105)),
                                    SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .build());
                    break;
                case 2:
                    drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(-41.75, -63.00, Math.toRadians(90.00)))
                            .lineToSplineHeading(new Pose2d(x2, -y2, Math.toRadians(angle2)),
                                    SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .build());
                    break;
                case 3:
                    drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(-41.75, -63.00, Math.toRadians(90.00)))
                            .splineToSplineHeading(new Pose2d(x3, -y3, Math.toRadians(angle3)), Math.toRadians(50),
                                    SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .build());
                    break;
            }

            currentState = State.EJECT_AND_PARK;
        }
        if(currentState == State.EJECT_AND_PARK && !drive.isBusy()) {
            drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .back(15)
                    .lineToLinearHeading(new Pose2d(-73, -60, Math.toRadians(0)))
                    .build());
            currentState = State.FINISHED;
        }
    }
}
