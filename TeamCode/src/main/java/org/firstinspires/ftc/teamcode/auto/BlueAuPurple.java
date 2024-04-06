package org.firstinspires.ftc.teamcode.auto;

import android.util.Size;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.CV.ZoneDetectionProcessorRight;
import org.firstinspires.ftc.teamcode.auto.util.AutoBaseOpmode;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.wpilib.MedianFilter;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name="Blue Au Purple Only", group = "Auto")
public class BlueAuPurple extends AutoBaseOpmode {

    private ZoneDetectionProcessorRight processor;
    private VisionPortal portal;

    public enum PurplePixState {
        WAIT_FOR_START,
        MOVE_TO_PROP,
        EJECT_AND_MOVE_TO_STACK,
        WAIT_FOR_FINISH,
        FINISHED
    }
    public static PurplePixState currentPurplePixState = PurplePixState.WAIT_FOR_START;

    public static boolean red = false;
    public static int zone = 1;


    public static double waitTime = 1.5;
    public static double x1 = -48.5;
    public static double y1 = 36;
    public static double x2 = -35;
    public static double y2 = 36;
    public static double x3 = -29;
    public static double y3 = 39;
    public static double angle2 = 80;
    public static double angle3 = 60;
    public static double pixelX = -56.5;
    public static double pixelY = 36;

    public enum WhiteStackState {
        WAIT_FOR_START,
        TOPPLE_STACK1,
        TOPPLE_STACK2,
        PICKUP_PIXELS,
        WAIT_FOR_INTAKE,
        EJECT,
        WAIT_FOR_EJECT,
        FINISHED
    }
    public static WhiteStackState currentWhiteStackState = WhiteStackState.WAIT_FOR_START;

    public static double intakeInitialPower = 0;
    public static double intakeKnockPower = 0.7;
    public static double intakeFinalPower = 0.6;
    public static double moveForwardDistance = 3.5;
    public static double moveBackDistance = 6;
    public static double pickUpPathSpeed = 20;
    public static double moveBackwardsSpeed = 10;
    public static double initialForwardSpeed = 15;
    public static double intakeWaitTime = 1000;


    private ElapsedTime intakeWaitTimer;


    public static int sampleSize = 15;
    private final MedianFilter filter = new MedianFilter(sampleSize);

    private int pixelCount = 0;

    public static double FIRST = 123;
    public static double SECOND = 105;
    public static double tolerance = 4;


    public enum ToBackdropState {
        WAIT_FOR_START,
        MOVE_TO_DETECT_POS,
        CHECK_FOR_BOT,
        MOVE_TO_BACKDROP,
        DEPOSIT,
        WAIT_FOR_FINISH,
        FINISHED
    }
    public static ToBackdropState currentToBackdropState = ToBackdropState.WAIT_FOR_START;

    public static double botDetectionThreshold = 65;
    public static double depositWaitTime = 500;
    public static double trussPixelSideX = -30;
    public static double bottomPathY = 66;
    public static double lineToX = 24;
    public static double lineToY = 64;
    public static double waitingX = 38;
    public static double waitingY = 64;
    public static double waitingHeading = 75;
    public static double backdropX = 48.5;
    public static double backdropY1 = 31;
    public static double backdropY2 = 37;
    public static double backdropY3 = 44;

    private ElapsedTime depositWaitTimer;

    @Override
    public void init(){
        super.init();
        depositWaitTimer = new ElapsedTime();
        currentPurplePixState = PurplePixState.WAIT_FOR_START;
        if(red)
            drive.setPoseEstimate(new Pose2d(-41.75, -63.00, Math.toRadians(90.00)));
        else
            drive.setPoseEstimate(new Pose2d(-41.75, 63.00, Math.toRadians(-90.00)));

        processor = new ZoneDetectionProcessorRight(false, false);
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
        zone = processor.getZone();
        schedule(armSubsystem.intake());
        schedule(intakeSubsystem.setStack1(0.5));
        schedule(intakeSubsystem.setStack2(0.5));
        depositWaitTimer.reset();
        currentPurplePixState = PurplePixState.MOVE_TO_PROP;
    }

    @Override
    public void loop() {
        super.loop();
        drive.update();
        liftSubsystem.periodic();

        if(currentPurplePixState == PurplePixState.MOVE_TO_PROP && depositWaitTimer.milliseconds() > 3000){
            zone = processor.getZone();
            portal.close();
            if(red){
                switch(zone) {
                    case 1:
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(-41.75, -63.00, Math.toRadians(90.00)))
                                .lineToSplineHeading(new Pose2d(x1, -y1, Math.toRadians(90.00)),
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
            }else{
                switch(zone) {
                    case 1:
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(-41.75, 63.00, Math.toRadians(-90.00)))
                                .splineToSplineHeading(new Pose2d(x3, y3, Math.toRadians(-angle3)), Math.toRadians(-50),
                                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build());
                        break;
                    case 2:
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(-41.75, 63.00, Math.toRadians(-90.00)))
                                .lineToSplineHeading(new Pose2d(x2, y2, Math.toRadians(-angle2)),
                                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build());
                        break;
                    case 3:
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(-41.75, 63.00, Math.toRadians(-90.00)))
                                .lineToSplineHeading(new Pose2d(x1, y1, Math.toRadians(-90.00)),
                                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build());
                        break;
                }
            }
            currentPurplePixState = PurplePixState.EJECT_AND_MOVE_TO_STACK;
        }

        //could replace isBusy with a posEstimate check if need efficiency
        if(currentPurplePixState == PurplePixState.EJECT_AND_MOVE_TO_STACK && !drive.isBusy()) {
            schedule(intakeSubsystem.setStack1(0.25));
            schedule(intakeSubsystem.setStack2(0.25));
            drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .waitSeconds(1)
                    .back(15,
                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build());

            currentPurplePixState = PurplePixState.FINISHED;
        }
    }
}
