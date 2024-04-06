package org.firstinspires.ftc.teamcode.auto;

import android.util.Size;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.CV.ZoneDetectionProcessorLeft;
import org.firstinspires.ftc.teamcode.auto.util.AutoBaseOpmode;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.util.wpilib.MedianFilter;
import org.firstinspires.ftc.teamcode.util.commands.DelayedCommand;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name="Red Au Test", group = "Auto")
public class RedAuCycle extends AutoBaseOpmode {

    private ZoneDetectionProcessorLeft processor;
    private VisionPortal portal;

    public enum PurplePixState {
        WAIT_FOR_START,
        MOVE_TO_PROP,
        EJECT_AND_MOVE_TO_STACK,
        WAIT_FOR_FINISH,
        FINISHED
    }
    public static PurplePixState currentPurplePixState = PurplePixState.WAIT_FOR_START;

    public static boolean red = true;
    public static int zone = 1;


    public static double waitTime = 1.5;
    public static double x1 = -47;
    public static double y1 = 35;
    public static double x2 = -35;
    public static double y2 = 36;
    public static double x3 = -29;
    public static double y3 = 38;
    public static double angle2 = 80;
    public static double angle3 = 60;
    public static double pixelX = -56.5;
    public static double pixelY = 36.5;

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
    public static double moveForwardDistance = 3;
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
    public static double bottomPathY = 56;
    public static double lineToX = 24;
    public static double lineToY = 54;
    public static double waitingX = 38;
    public static double waitingY = 52;
    public static double waitingHeading = 75;
    public static double backdropX = 50;
    public static double backdropY1 = 25;
    public static double backdropY2 = 32;
    public static double backdropY3 = 40;

    private ElapsedTime depositWaitTimer;

    public enum ToStackState {
        WAIT_FOR_START,
        MOVE_TO_STACKS,
        RUN_INTAKE,
        PICKUP,
        EJECT_AND_DETECT_POS,
        FINISHED
    }
    public static ToStackState currentToStackState = ToStackState.WAIT_FOR_START;

    public static double pixelPathSpeed = 30;
    public static double toBottomRowX = 30;
    public static double toBottomPathY = 52;
    public static double returnLineToX = -26;
    public static double returnLineToY = 56;
    public static double pixelPathStartX = -48;
    public static double pixelPathY = 36;
    public static double pixelPathEndX = -59;



    @Override
    public void init(){
        super.init();
        depositWaitTimer = new ElapsedTime();
        intakeWaitTimer = new ElapsedTime();
        currentPurplePixState = PurplePixState.WAIT_FOR_START;
        if(red)
            drive.setPoseEstimate(new Pose2d(-41.75, -63.00, Math.toRadians(90.00)));
        else
            drive.setPoseEstimate(new Pose2d(-41.75, 63.00, Math.toRadians(-90.00)));

        processor = new ZoneDetectionProcessorLeft(true, false);
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
        portal.close();
        currentPurplePixState = PurplePixState.MOVE_TO_PROP;
    }

    @Override
    public void loop() {
        super.loop();
        drive.update();
        liftSubsystem.periodic();

        //TODO: delete testing above
        if(currentPurplePixState == PurplePixState.MOVE_TO_PROP){
            schedule(armSubsystem.intake());
            schedule(intakeSubsystem.setStack1(0.4));
            schedule(intakeSubsystem.setStack2(0.4));
            if(red){
                switch(zone) {
                    case 1:
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(-41.75, -63.00, Math.toRadians(90.00)))
                                .lineToSplineHeading(new Pose2d(x1, -y1, Math.toRadians(90.00)))
                                .build());
                        break;
                    case 2:
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(-41.75, -63.00, Math.toRadians(90.00)))
                                .lineToSplineHeading(new Pose2d(x2, -y2, Math.toRadians(angle2)))
                                .build());
                        break;
                    case 3:
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(-41.75, -63.00, Math.toRadians(90.00)))
                                .splineToSplineHeading(new Pose2d(x3, -y3, Math.toRadians(angle3)), Math.toRadians(50))
                                .build());
                        break;
                }
            }else{
                switch(zone) {
                    case 1:
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(-41.75, 63.00, Math.toRadians(-90.00)))
                                .splineToSplineHeading(new Pose2d(x3, y3, Math.toRadians(-angle3)), Math.toRadians(-50))
                                .build());
                        break;
                    case 2:
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(-41.75, 63.00, Math.toRadians(-90.00)))
                                .lineToSplineHeading(new Pose2d(x2, y2, Math.toRadians(-angle2)))
                                .build());
                        break;
                    case 3:
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(-41.75, 63.00, Math.toRadians(-90.00)))
                                .lineToSplineHeading(new Pose2d(x1, y1, Math.toRadians(-90.00)))
                                .build());
                        break;
                }
            }
            currentPurplePixState = PurplePixState.EJECT_AND_MOVE_TO_STACK;
        }

        //could replace isBusy with a posEstimate check if need efficiency
        if(currentPurplePixState == PurplePixState.EJECT_AND_MOVE_TO_STACK && !drive.isBusy()) {
            //intake fsm
            //signal with a displacement marker that changes an enum
            schedule(intakeSubsystem.runIntake(-0.5));
            schedule(new DelayedCommand(intakeSubsystem.runIntake(0), (long)waitTime*1000));
            schedule(new DelayedCommand(armSubsystem.intake(),1000));
            schedule(new DelayedCommand(intakeSubsystem.setStack1(IntakeSubsystem.servoHighPosition +0.05),1000));
            schedule(new DelayedCommand(intakeSubsystem.setStack2(IntakeSubsystem.servoHighPosition +0.05),1000));

            //drive fsm
            if(red){
                drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .back(15,
                                SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(40))
                        .forward(0.5)
                        .splineTo(new Vector2d(pixelX - moveForwardDistance, pixelY), Math.toRadians(180.00),
                                SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(70))
                        .build());
            }else{
                drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .back(15,
                                SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(40))
                        .forward(0.5)
                        .splineTo(new Vector2d(pixelX - moveForwardDistance, pixelY), Math.toRadians(180.00),
                                SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(70))
                        .build());
            }
            currentPurplePixState = PurplePixState.FINISHED;
            currentWhiteStackState = WhiteStackState.TOPPLE_STACK2;
        }

        if(currentPurplePixState == PurplePixState.WAIT_FOR_FINISH && !drive.isBusy()){
            currentPurplePixState = PurplePixState.FINISHED;
            currentWhiteStackState = WhiteStackState.TOPPLE_STACK1;
        }

        if (currentWhiteStackState == WhiteStackState.TOPPLE_STACK1) {
            if (red) {
                drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(pixelX, -pixelY, Math.toRadians(180.00)))
                        .lineToLinearHeading(new Pose2d(pixelX - moveForwardDistance, -pixelY, Math.toRadians(180.00)),
                                SampleMecanumDrive.getVelocityConstraint(initialForwardSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(initialForwardSpeed))
                        .build());
            } else {
                drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(pixelX, pixelY, Math.toRadians(180.00)))
                        .lineToLinearHeading(new Pose2d(pixelX - moveForwardDistance, pixelY, Math.toRadians(180.00)),
                                SampleMecanumDrive.getVelocityConstraint(initialForwardSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(initialForwardSpeed))
                        .build());
            }
            currentWhiteStackState = WhiteStackState.TOPPLE_STACK2;
        }
        if(currentWhiteStackState == WhiteStackState.TOPPLE_STACK2 && !drive.isBusy()){
            intakeWaitTimer.reset();
            schedule(intakeSubsystem.runIntake(intakeKnockPower));
            if (red) {
                drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(pixelX, -pixelY, Math.toRadians(180.00)))
                        .lineToLinearHeading(new Pose2d(pixelX + moveBackDistance, -pixelY, Math.toRadians(180.00)),
                                SampleMecanumDrive.getVelocityConstraint(moveBackwardsSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .forward(2.5)
                        .build());
            } else {
                drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(pixelX, pixelY, Math.toRadians(180.00)))
                        .lineToLinearHeading(new Pose2d(pixelX + moveBackDistance, pixelY, Math.toRadians(180.00)),
                                SampleMecanumDrive.getVelocityConstraint(moveBackwardsSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .forward(2.5)
                        .build());
            }
            currentWhiteStackState = WhiteStackState.PICKUP_PIXELS;
        }
        if (currentWhiteStackState == WhiteStackState.PICKUP_PIXELS && intakeWaitTimer.milliseconds()>500) {
            schedule(intakeSubsystem.setStack1(IntakeSubsystem.servoLowPosition));
            schedule(intakeSubsystem.setStack2(IntakeSubsystem.servoLowPosition));
            schedule(intakeSubsystem.runIntake(intakeFinalPower));
            currentWhiteStackState = WhiteStackState.WAIT_FOR_INTAKE;
        }
        if (currentWhiteStackState == WhiteStackState.WAIT_FOR_INTAKE && !drive.isBusy()) {
            currentWhiteStackState = WhiteStackState.FINISHED;
            currentToBackdropState = ToBackdropState.MOVE_TO_DETECT_POS;
        }
//        if (currentWhiteStackState == WhiteStackState.EJECT && intakeWaitTimer.milliseconds()>intakeWaitTime) {
//            schedule(intakeSys.setStack1(IntakeSys.intakeServoHighPosition+0.05));
//            schedule(intakeSys.setStack2(IntakeSys.intakeServoHighPosition+0.05));
//            schedule(intakeSys.runIntake(-IntakeSys.intakeOutPower));
//            schedule(new DelayedCommand(intakeSys.runIntake(-IntakeSys.intakeOutPower),500));
//            intakeWaitTimer.reset();
//            currentWhiteStackState = WhiteStackState.WAIT_FOR_EJECT;
//        }
//        if (currentWhiteStackState == WhiteStackState.WAIT_FOR_EJECT && intakeWaitTimer.milliseconds()>500) {
//            schedule(intakeSys.runIntake(0));
//            currentWhiteStackState = WhiteStackState.FINISHED;
//            currentToBackdropState = ToBackdropState.MOVE_TO_DETECT_POS;
//        }

        if(currentToBackdropState == ToBackdropState.MOVE_TO_DETECT_POS){
            schedule(intakeSubsystem.setStack1(IntakeSubsystem.servoHighPosition +0.05));
            schedule(intakeSubsystem.setStack2(IntakeSubsystem.servoHighPosition +0.05));
            schedule(intakeSubsystem.runIntake(-IntakeSubsystem.intakeOutPower));
            schedule(new DelayedCommand(intakeSubsystem.runIntake(0),500));
            schedule(boxSubsystem.close());
            if(red){
                drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .back(1)
                        .splineToConstantHeading(new Vector2d(trussPixelSideX, -bottomPathY), Math.toRadians(0))
                        .splineTo(new Vector2d(lineToX, -lineToY), Math.toRadians(0.00))
                        .lineToSplineHeading(new Pose2d(waitingX, -waitingY, Math.toRadians(180+waitingHeading)),
                                SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .waitSeconds(0.5)
                        .build());
            }else{
                drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .back(1)
                        .splineToConstantHeading(new Vector2d(trussPixelSideX, bottomPathY), Math.toRadians(0))
                        .splineTo(new Vector2d(lineToX, lineToY), Math.toRadians(0.00))
                        .lineToSplineHeading(new Pose2d(waitingX, waitingY, Math.toRadians(180-waitingHeading)),
                                SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .waitSeconds(0.5)
                        .build());
            }
            currentToBackdropState = ToBackdropState.CHECK_FOR_BOT;
        }
        if(currentToBackdropState == ToBackdropState.CHECK_FOR_BOT && !drive.isBusy()){
            if(localizerSubsystem.getBl() > botDetectionThreshold){
                currentToBackdropState = ToBackdropState.MOVE_TO_BACKDROP;
            }
        }
        if(currentToBackdropState == ToBackdropState.MOVE_TO_BACKDROP){
            schedule(liftSubsystem.goTo(LiftSubsystem.LOW));
            schedule(new DelayedCommand(armSubsystem.deposit(),500));
            if(red){
                switch(zone){
                    case 1:
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(waitingX, -bottomPathY, Math.toRadians(180+waitingHeading)))
                                .lineToLinearHeading(new Pose2d(backdropX, -backdropY1, Math.toRadians(180)))
                                .build());
                        break;
                    case 2:
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(waitingX, -bottomPathY, Math.toRadians(180+waitingHeading)))
                                .lineToLinearHeading(new Pose2d(backdropX, -backdropY2, Math.toRadians(180)))
                                .build());
                        break;
                    case 3:
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(waitingX, -bottomPathY, Math.toRadians(180+waitingHeading)))
                                .lineToLinearHeading(new Pose2d(backdropX, -backdropY3, Math.toRadians(180)))
                                .build());
                        break;
                }
            }else{
                switch(zone){
                    case 1:
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(waitingX, bottomPathY, Math.toRadians(180-waitingHeading)))
                                .lineToLinearHeading(new Pose2d(backdropX, backdropY3, Math.toRadians(180)))
                                .build());
                        break;
                    case 2:
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(waitingX, bottomPathY, Math.toRadians(180-waitingHeading)))
                                .lineToLinearHeading(new Pose2d(backdropX, backdropY2, Math.toRadians(180)))
                                .build());
                        break;
                    case 3:
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(waitingX, bottomPathY, Math.toRadians(180-waitingHeading)))
                                .lineToLinearHeading(new Pose2d(backdropX, backdropY1, Math.toRadians(180)))
                                .build());
                        break;
                }
            }
            currentToBackdropState = ToBackdropState.DEPOSIT;
        }
        if(currentToBackdropState == ToBackdropState.DEPOSIT && !drive.isBusy()){
            schedule(boxSubsystem.intake());
            depositWaitTimer.reset();
            //TODO: add apriltag relocalization
            currentToBackdropState = ToBackdropState.WAIT_FOR_FINISH;
        }
        if(currentToBackdropState == ToBackdropState.WAIT_FOR_FINISH && depositWaitTimer.milliseconds() > depositWaitTime){
            currentToBackdropState = ToBackdropState.FINISHED;
            currentToStackState = ToStackState.MOVE_TO_STACKS;
        }
        if(currentToStackState == ToStackState.MOVE_TO_STACKS){
            schedule(liftSubsystem.goTo(LiftSubsystem.NONE));
            schedule(armSubsystem.intake());
            if(red){
                drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(50, -46, Math.toRadians(180)))
                        .splineToConstantHeading(new Vector2d(toBottomRowX, -toBottomPathY), Math.toRadians(180))
                        .splineTo(new Vector2d(returnLineToX, -returnLineToY), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(pixelPathStartX, -pixelPathY), Math.toRadians(180))
                        .lineToSplineHeading(new Pose2d(pixelPathEndX, -pixelPathY, Math.toRadians(180)),
                                SampleMecanumDrive.getVelocityConstraint(pixelPathSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build());
            }else{
                drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(50, -46, Math.toRadians(180)))
                        .splineToConstantHeading(new Vector2d(toBottomRowX, toBottomPathY), Math.toRadians(180))
                        .splineTo(new Vector2d(returnLineToX, returnLineToY), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(pixelPathStartX, pixelPathY), Math.toRadians(180))
                        .lineToSplineHeading(new Pose2d(pixelPathEndX, pixelPathY, Math.toRadians(180)),
                                SampleMecanumDrive.getVelocityConstraint(pixelPathSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build());
            }
            currentToStackState = ToStackState.RUN_INTAKE;
        }
        if(currentToStackState == ToStackState.RUN_INTAKE && drive.getPoseEstimate().getX() < pixelPathStartX+4){
            schedule(intakeSubsystem.runIntake(IntakeSubsystem.intakeInPower));
            schedule(intakeSubsystem.setStack1(IntakeSubsystem.servoLowPosition));
            schedule(intakeSubsystem.setStack2(IntakeSubsystem.servoLowPosition));
            currentToStackState = ToStackState.PICKUP;
        }
        if(currentToStackState == ToStackState.PICKUP && !drive.isBusy()){
            drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .back(7)
                    .forward(3)
                    .build());
            currentToStackState = ToStackState.EJECT_AND_DETECT_POS;
        }
        if(currentToStackState == ToStackState.EJECT_AND_DETECT_POS && !drive.isBusy()){
            schedule(intakeSubsystem.setStack1(IntakeSubsystem.servoHighPosition +0.05));
            schedule(intakeSubsystem.setStack2(IntakeSubsystem.servoHighPosition +0.05));
            schedule(intakeSubsystem.runIntake(-IntakeSubsystem.intakeOutPower));
            schedule(new DelayedCommand(intakeSubsystem.runIntake(0),500));
            schedule(boxSubsystem.close());
            if(red){
                drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .back(1)
                        .splineToConstantHeading(new Vector2d(trussPixelSideX, -bottomPathY), Math.toRadians(0))
                        .splineTo(new Vector2d(lineToX, -lineToY), Math.toRadians(0.00))
                        .lineToSplineHeading(new Pose2d(waitingX, -waitingY, Math.toRadians(180+waitingHeading)),
                                SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .waitSeconds(0.5)
                        .build());
            }else{
                drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .back(1)
                        .splineToConstantHeading(new Vector2d(trussPixelSideX, bottomPathY), Math.toRadians(0))
                        .splineTo(new Vector2d(lineToX, lineToY), Math.toRadians(0.00))
                        .lineToSplineHeading(new Pose2d(waitingX, waitingY, Math.toRadians(180-waitingHeading)),
                                SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .waitSeconds(0.5)
                        .build());
            }
        }
    }
}
