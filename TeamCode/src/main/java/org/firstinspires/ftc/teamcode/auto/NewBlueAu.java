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
import org.firstinspires.ftc.teamcode.auto.CV.ZoneDetectionProcessorRight;
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
@Autonomous(name="New Blue Au", group = "Auto")
public class NewBlueAu extends AutoBaseOpmode {

    private HSVDetectionPipeline processor;
    private VisionPortal portal;

    public static int sampleSize = 5;
    private final HighestSampleFilter filter = new HighestSampleFilter(sampleSize);

    public enum PurplePixState {
        WAIT_FOR_START,
        MOVE_TO_PROP,
        PLACE_AND_RETURN_TO_START,
        WAIT_FOR_FINISH,
        CHECK_FOR_ALLIANCE_BOT,
        WAIT_FOR_MOVE,
        CHECK_AGAIN_FOR_ALLIANCE_BOT,
        DEPOSIT,
        DROP,
        PARK,
        FINISH

    }
    public static PurplePixState currentPurplePixState = PurplePixState.WAIT_FOR_START;

    public static boolean red = false;
    public static int zone = 2;


    public static double x1 = -46;
    public static double y1 = 37;
    public static double x2 = -35;
    public static double y2 = 35.5;
    public static double x3 = -29.5;
    public static double y3 = 39;
    public static double angle2 = 80;
    public static double angle3 = 60;
    public static double pixelX = -56.5;
    public static double pixelY = 36;

    public static int taps = 10;
    private final LinearFilter lowPass = LinearFilter.movingAverage(taps);

    private int pixelCount = 0;

    public static double FIRST = 123;
    public static double SECOND = 105;
    public static double tolerance = 4;

    private ElapsedTime depositWaitTimer;

    public static double backdropX = 50;
    public static double backdropY1 = 22;
    public static double backdropY2 = 28.75;
    public static double backdropY3 = 35.5;

    private double BLDistance;

    @Override
    public void init(){
        super.init();
        depositWaitTimer = new ElapsedTime();
        currentPurplePixState = PurplePixState.WAIT_FOR_START;
        if(red)
            drive.setPoseEstimate(new Pose2d(-41.75, -63.00, Math.toRadians(90.00)));
        else
            drive.setPoseEstimate(new Pose2d(-41.75, 63.00, Math.toRadians(-90.00)));

        processor = new HSVDetectionPipeline(Side.BLUE_FAR);
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
        currentPurplePixState = PurplePixState.MOVE_TO_PROP;
    }

    @Override
    public void loop() {
        telemetry.addData("zone", processor.getZone());
        super.loop();
        telemetry.addData("raw bl data", localizerSubsystem.getBl());
        drive.update();
        liftSubsystem.periodic();
        BLDistance = filter.calculate(lowPass.calculate(localizerSubsystem.getBl()));
        if(currentPurplePixState == PurplePixState.MOVE_TO_PROP && depositWaitTimer.milliseconds() > 2500){
            zone = processor.getZone();
            portal.close();
            schedule(intakeSubsystem.setStack1(IntakeSubsystem.servoHighPosition));
            schedule(intakeSubsystem.setStack2(IntakeSubsystem.servo2HighPosition));
            if(red){
                switch(zone) {
                    case 1:
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(-41.75, -63.00, Math.toRadians(90.00)))
                                .lineToSplineHeading(new Pose2d(x1, -y1, Math.toRadians(100)),
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
                                .lineToSplineHeading(new Pose2d(x1, y1, Math.toRadians(-105.00)),
                                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build());
                        break;
                }
            }
            currentPurplePixState = PurplePixState.PLACE_AND_RETURN_TO_START;
        }

        //could replace isBusy with a posEstimate check if need efficiency
        if(currentPurplePixState == PurplePixState.PLACE_AND_RETURN_TO_START && !drive.isBusy()) {
            drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(-45,60,Math.toRadians(180-2)),
                            SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .lineToLinearHeading(new Pose2d(-7,59,Math.toRadians(180-2)))
                    .build());

            schedule(boxSubsystem.close());
            currentPurplePixState = PurplePixState.WAIT_FOR_MOVE;
        }

        if(currentPurplePixState == PurplePixState.WAIT_FOR_MOVE && !drive.isBusy()){
            if(localizerSubsystem.getBl()>70){
                if(depositWaitTimer.seconds()<2) {
                    drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .waitSeconds(1.5)
                            .lineToSplineHeading(new Pose2d(38, 56, Math.toRadians(180-75)),
                                    SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .build());
                }else{
                    drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .lineToSplineHeading(new Pose2d(38, 56, Math.toRadians(180-75)),
                                    SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .build());
                }
                currentPurplePixState = PurplePixState.CHECK_AGAIN_FOR_ALLIANCE_BOT;
            }else{
                depositWaitTimer.reset();
            }
        }


        if(currentPurplePixState == PurplePixState.CHECK_AGAIN_FOR_ALLIANCE_BOT && !drive.isBusy()){
            if(localizerSubsystem.getBl() > 65){
                currentPurplePixState = PurplePixState.DEPOSIT;
            }else{
                depositWaitTimer.reset();
            }
        }

        if(currentPurplePixState == PurplePixState.DEPOSIT && depositWaitTimer.milliseconds()>1500){
            schedule(liftSubsystem.goTo(LiftSubsystem.LOW-300));
            schedule(new DelayedCommand(armSubsystem.deposit(),500));
            switch(zone){
                case 3:
                    drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(backdropX, backdropY1, Math.toRadians(180-1)))
                            .build());
                    break;
                case 2:
                    drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(backdropX, backdropY2, Math.toRadians(180-1)))
                            .build());
                    break;
                case 1:
                    drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(backdropX, backdropY3, Math.toRadians(180-1)))
                            .build());
                    break;
            }
            currentPurplePixState = PurplePixState.DROP;
        }
        if(currentPurplePixState == PurplePixState.DROP && !drive.isBusy()){
            schedule(new DelayedCommand(new InstantCommand(()-> boxSubsystem.release()),1000));
            schedule(new DelayedCommand(new InstantCommand(()-> boxSubsystem.release()),1000));
            depositWaitTimer.reset();
            currentPurplePixState = PurplePixState.PARK;
        }
        if(currentPurplePixState == PurplePixState.PARK && depositWaitTimer.milliseconds()>2000){
            schedule(liftSubsystem.goTo(LiftSubsystem.NONE));
            schedule(armSubsystem.intake());
            drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(50, 56, Math.toRadians(180)))
                    .build());
            currentPurplePixState = PurplePixState.FINISH;
        }



    }
}
