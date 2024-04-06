package org.firstinspires.ftc.teamcode.auto.pathPieces.backdropStart;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.auto.util.AutoBaseOpmode;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.util.commands.DelayedCommand;

@Config
@TeleOp(name="Purple Pixel Ba", group = "ZZZ")
public class PurplePixelBa extends AutoBaseOpmode {

    public enum State{
        WAIT_FOR_START,
        MOVE_TO_PROP,
        EJECT_AND_MOVE_TO_BACKDROP,
        DEPOSIT,
        WAIT_FOR_FINISH,
        FINISHED
    }
    public static State currentState = State.WAIT_FOR_START;

    public static boolean red = true;
    public static int zone = 1;

    public static double depositWaitTime = 500;
    public static double x1 = 6;
    public static double y1 = 42;
    public static double x2 = 28;
    public static double y2 = 24.5;
    public static double x3 = 23.5;
    public static double y3 = 45;
    public static double angle1 = 110;
    public static double backdropX = 50;
    public static double backdropY1 = 29;
    public static double backdropY2 = 36;
    public static double backdropY3 = 43;

    public static boolean testing = false;

    private ElapsedTime depositWaitTimer;

    @Override
    public void init(){
        super.init();
        depositWaitTimer = new ElapsedTime();
        if(red)
            drive.setPoseEstimate(new Pose2d(17.75, -63.00, Math.toRadians(90.00)));
        else
            drive.setPoseEstimate(new Pose2d(17.75, 63.00, Math.toRadians(-90.00)));
    }

    @Override
    public void loop() {
        super.loop();
        drive.update();

        //drive fsm
        if(gamepad1.right_bumper && !testing){
            testing = true;
            currentState = State.MOVE_TO_PROP;
        }
        if(gamepad1.left_bumper && testing){
            testing = false;
            if(red){
                drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(17.75, -63.00, Math.toRadians(90.00)))
                        .build());
            }else{
                drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(17.75, 63.00, Math.toRadians(-90.00)))
                        .build());
            }
        }





        //TODO: delete testing above
        if(currentState == State.MOVE_TO_PROP){
            schedule(boxSubsystem.close());
            if(red){
                switch(zone) {
                    case 1:
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(17.75, -63.00, Math.toRadians(90.00)))
                                .splineToSplineHeading(new Pose2d(x1, -y1, Math.toRadians(angle1)), Math.toRadians(angle1+10))
                                .build());
                        break;
                    case 2:
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(17.75, -63.00, Math.toRadians(90.00)))
                                .lineToSplineHeading(new Pose2d(x2, -y2, Math.toRadians(180)))
                                .build());
                        break;
                    case 3:
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(17.75, -63.00, Math.toRadians(90.00)))
                                .lineToSplineHeading(new Pose2d(x3, -y3, Math.toRadians(90)))
                                .build());
                        break;
                }
            }else{
                switch(zone) {
                    case 1:
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(17.75, 63.00, Math.toRadians(-90.00)))
                                .lineToSplineHeading(new Pose2d(x3, y3, Math.toRadians(-90)))
                                .build());
                        break;
                    case 2:
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(17.75, 63.00, Math.toRadians(-90.00)))
                                .lineToSplineHeading(new Pose2d(x2, y2, Math.toRadians(180)))
                                .build());
                        break;
                    case 3:
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(17.75, 63.00, Math.toRadians(-90.00)))
                                .splineToSplineHeading(new Pose2d(x1, y1, Math.toRadians(-angle1)), Math.toRadians(-(angle1+10)))
                                .build());
                        break;
                }
            }
            currentState = State.EJECT_AND_MOVE_TO_BACKDROP;
        }

        //could replace isBusy with a posEstimate check if need efficiency
        if(currentState == State.EJECT_AND_MOVE_TO_BACKDROP && !drive.isBusy()) {
            schedule(intakeSubsystem.runIntake(-0.5));
            schedule(new DelayedCommand(intakeSubsystem.runIntake(-0.5), 500));
            schedule(new DelayedCommand(liftSubsystem.goTo(LiftSubsystem.LOW),500));
            schedule(new DelayedCommand(armSubsystem.deposit(),500));
            //drive fsm
            if(red){
                switch(zone) {
                    case 1:
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .waitSeconds(.25)
                                .lineToSplineHeading(new Pose2d(backdropX, -backdropY1, Math.toRadians(180)))
                                .build());
                        break;
                    case 2:
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .waitSeconds(.25)
                                .lineToSplineHeading(new Pose2d(backdropX, -backdropY2, Math.toRadians(180)))
                                .build());
                        break;
                    case 3:
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .waitSeconds(.25)
                                .lineToSplineHeading(new Pose2d(backdropX, -backdropY3, Math.toRadians(180)))
                                .build());
                        break;
                }
            }else{
                switch(zone) {
                    case 1:
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .waitSeconds(.25)
                                .lineToSplineHeading(new Pose2d(backdropX, backdropY3, Math.toRadians(180)))
                                .build());
                        break;
                    case 2:
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .waitSeconds(.25)
                                .lineToSplineHeading(new Pose2d(backdropX, backdropY2, Math.toRadians(180)))
                                .build());
                        break;
                    case 3:
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .waitSeconds(.25)
                                .lineToSplineHeading(new Pose2d(backdropX, backdropY1, Math.toRadians(180)))
                                .build());
                        break;
                }
            }
            currentState = State.DEPOSIT;
        }
        if(currentState == State.DEPOSIT && !drive.isBusy()){
            schedule(boxSubsystem.intake());
            depositWaitTimer.reset();
            currentState = State.WAIT_FOR_FINISH;
        }
        if(currentState == State.WAIT_FOR_FINISH && depositWaitTimer.milliseconds()>depositWaitTime){
            currentState = State.FINISHED;
        }

    }

    public static void run(int detectedZone, boolean redSide){
        zone = detectedZone;
        red = redSide;
        currentState = State.MOVE_TO_PROP;
    }
    public static boolean isFinished(){
        return currentState == State.FINISHED;
    }
}
