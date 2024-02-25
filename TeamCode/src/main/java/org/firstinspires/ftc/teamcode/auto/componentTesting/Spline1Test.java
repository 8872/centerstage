package org.firstinspires.ftc.teamcode.auto.componentTesting;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.auto.util.AutoBaseOpmode;
import org.firstinspires.ftc.teamcode.util.commands.DelayedCommand;

@Config
@TeleOp(name="Spline 1 Test", group = "ZZZ")
public class Spline1Test extends AutoBaseOpmode {

    public enum State{
        WAIT_FOR_START,
        MOVE_TO_PROP,
        EJECT_AND_MOVE_TO_STACK,
        WAIT_FOR_FINISH,
        FINISHED
    }
    public static State currentState = State.WAIT_FOR_START;

    public static boolean red = true;
    public static int zone = 3;


    public static double x1 = -47.5;
    public static double y1 = 45;
    public static double x2 = -52;
    public static double y2 = 24.5;
    public static double x3 = -30;
    public static double y3 = 42;
    public static double angle3 = 70;
    public static double pixelX = -56;
    public static double pixelY = 36;

    public static boolean testing = false;

    @Override
    public void init(){
        super.init();
        testing = false;
        currentState = State.WAIT_FOR_START;
        if(red)
            drive.setPoseEstimate(new Pose2d(-41.75, -63.00, Math.toRadians(90.00)));
        else
            drive.setPoseEstimate(new Pose2d(-41.75, 63.00, Math.toRadians(-90.00)));
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
                        .lineToLinearHeading(new Pose2d(-41.75, -63.00, Math.toRadians(90.00)))
                        .build());
            }else{
                drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-41.75, 63.00, Math.toRadians(-90.00)))
                        .build());
            }
        }



        //TODO: delete testing above
        if(currentState == State.MOVE_TO_PROP){
            drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(x1, y1, Math.toRadians(-90.00)))
                    .build());
            currentState = State.WAIT_FOR_FINISH;
        }

        if(currentState == State.WAIT_FOR_FINISH && !drive.isBusy()){
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
