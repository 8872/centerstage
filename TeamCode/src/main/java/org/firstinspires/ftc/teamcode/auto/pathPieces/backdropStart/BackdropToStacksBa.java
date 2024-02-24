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
@Disabled
@TeleOp(name="To Stacks Ba", group = "ZZZ")
public class BackdropToStacksBa extends AutoBaseOpmode {

    public enum State{
        WAIT_FOR_START,
        MOVE_TO_STACKS,
        WAIT_FOR_FINISH,
        FINISHED
    }
    public static State currentState = State.WAIT_FOR_START;

    public static boolean red = true;
    public static double middlePathX = 22.00;
    public static double middlePathY = 12.00;
    public static double lineToStackX = -56.00;

    public static boolean testing = false;

    @Override
    public void init(){
        //TODO: remove setPosEstimate
        if(red)
            drive.setPoseEstimate(new Pose2d(50, -43, Math.toRadians(180)));
        else
            drive.setPoseEstimate(new Pose2d(50, 43, Math.toRadians(180)));
    }

    @Override
    public void loop() {
        super.loop();

        if(gamepad1.right_bumper && !testing){
            testing = true;
            currentState = State.MOVE_TO_STACKS;
        }
        if(currentState == State.MOVE_TO_STACKS){
            schedule(liftSys.goTo(LiftSys.NONE));
            schedule(armSys.intake());
            if(red){
                drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineToConstantHeading(new Vector2d(middlePathX, -middlePathY), Math.toRadians(180.00))
                        .splineToConstantHeading(new Vector2d(lineToStackX, -middlePathY), Math.toRadians(180.00))
                        .build());
            }else{
                drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineToConstantHeading(new Vector2d(middlePathX, middlePathY), Math.toRadians(180.00))
                        .splineToConstantHeading(new Vector2d(lineToStackX, middlePathY), Math.toRadians(180.00))
                        .build());
            }
            currentState = State.WAIT_FOR_FINISH;
        }
        if(currentState == State.WAIT_FOR_FINISH && !drive.isBusy()){
            currentState = State.FINISHED;
        }
    }

    public static void run(boolean redSide){
        red = redSide;
        currentState = State.MOVE_TO_STACKS;
    }
    public static boolean isFinished(){
        return currentState == State.FINISHED;
    }
}
