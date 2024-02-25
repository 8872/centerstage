package org.firstinspires.ftc.teamcode.auto.pathPieces.backdropStart;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.auto.util.AutoBaseOpmode;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSys;
import org.firstinspires.ftc.teamcode.subsystem.LiftSys;
import org.firstinspires.ftc.teamcode.util.commands.DelayedCommand;

@Config
@TeleOp(name="To Backdrop Ba", group = "ZZZ")
public class StacksToBackdropBa extends AutoBaseOpmode {

    public enum State{
        WAIT_FOR_START,
        MOVE_TO_BACKDROP,
        DEPOSIT,
        WAIT_FOR_FINISH,
        FINISHED
    }
    public static State currentState = State.WAIT_FOR_START;

    public static double depositWaitTime = 500;
    public static boolean red = true;
    public static double middlePathY = 12.00;
    public static double lineToBackdropX = 22.00;
    public static double backdropX = 50;
    public static double backdropY = 29;

    public static boolean testing = false;

    private ElapsedTime depositWaitTimer;

    @Override
    public void init(){
        super.init();
        depositWaitTimer = new ElapsedTime();
        //TODO: remove setPosEstimate
        if(red)
            drive.setPoseEstimate(new Pose2d(-56.00, -12.00, Math.toRadians(180)));
        else
            drive.setPoseEstimate(new Pose2d(-56.00, 12.00, Math.toRadians(180)));
    }

    @Override
    public void loop() {
        super.loop();
        drive.update();

        if(gamepad1.right_bumper && !testing){
            testing = true;
            currentState = State.MOVE_TO_BACKDROP;
        }
        if(currentState == State.MOVE_TO_BACKDROP){
            schedule(boxSys.close());
            if(red){
                drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .back(1)
                        .splineTo(new Vector2d(lineToBackdropX, -middlePathY), Math.toRadians(180.00))
                        .splineToConstantHeading(new Vector2d(backdropX, -backdropY), Math.toRadians(180.00))
                        .build());
            }else{
                drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineTo(new Vector2d(lineToBackdropX, middlePathY), Math.toRadians(180.00))
                        .splineToConstantHeading(new Vector2d(backdropX, backdropY), Math.toRadians(180.00))
                        .build());
            }
            currentState = State.DEPOSIT;
        }
        if(drive.getPoseEstimate().getX() > 24 && currentState == State.DEPOSIT){
            schedule(liftSys.goTo(LiftSys.LOW));
            schedule(armSys.deposit());
            currentState = State.DEPOSIT;
        }
        if(currentState == State.DEPOSIT && !drive.isBusy()){
            schedule(boxSys.intake());
            depositWaitTimer.reset();
            currentState = State.WAIT_FOR_FINISH;
        }
        if(currentState == State.WAIT_FOR_FINISH && depositWaitTimer.milliseconds() > depositWaitTime){
            currentState = State.FINISHED;
        }
    }

    public static void run(boolean redSide){
        red = redSide;
        currentState = State.MOVE_TO_BACKDROP;
    }
    public static boolean isFinished(){
        return currentState == State.FINISHED;
    }
}
