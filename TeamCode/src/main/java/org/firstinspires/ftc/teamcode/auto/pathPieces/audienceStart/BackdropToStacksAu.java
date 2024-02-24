package org.firstinspires.ftc.teamcode.auto.pathPieces.audienceStart;

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
@TeleOp(name="To Stacks Au", group = "ZZZ")
public class BackdropToStacksAu extends AutoBaseOpmode {

    public enum State{
        WAIT_FOR_START,
        MOVE_TO_STACKS,
        RUN_INTAKE,
        WAIT_FOR_FINISH,
        FINISHED
    }
    public static State currentState = State.WAIT_FOR_START;

    public static boolean red = true;

    public static double pixelPathSpeed = 10;
    public static double toBottomRowX = 30;
    public static double bottomPathY = 60;
    public static double lineToX = -26;
    public static double pixelPathStartX = -44;
    public static double pixelPathY = 36;
    public static double pixelPathEndX = -60;

    public static boolean testing = false;

    @Override
    public void init(){
        //TODO: remove setPosEstimate
        if(red)
            drive.setPoseEstimate(new Pose2d(50, -44, Math.toRadians(180)));
        else
            drive.setPoseEstimate(new Pose2d(50, 44, Math.toRadians(180)));
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
                drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(50, -46, Math.toRadians(180)))
                        .splineToConstantHeading(new Vector2d(toBottomRowX, -bottomPathY), Math.toRadians(180))
                        .splineTo(new Vector2d(lineToX, -bottomPathY), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(pixelPathStartX, -pixelPathY), Math.toRadians(180))
                        .lineToSplineHeading(new Pose2d(pixelPathEndX, -pixelPathY, Math.toRadians(180)),
                                SampleMecanumDrive.getVelocityConstraint(pixelPathSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build());
            }else{
                drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(50, -46, Math.toRadians(180)))
                        .splineToConstantHeading(new Vector2d(toBottomRowX, bottomPathY), Math.toRadians(180))
                        .splineTo(new Vector2d(lineToX, bottomPathY), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(pixelPathStartX, pixelPathY), Math.toRadians(180))
                        .lineToSplineHeading(new Pose2d(pixelPathEndX, pixelPathY, Math.toRadians(180)),
                                SampleMecanumDrive.getVelocityConstraint(pixelPathSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build());
            }
            currentState = State.RUN_INTAKE;
        }
        if(currentState == State.RUN_INTAKE && drive.getPoseEstimate().getX() < pixelPathStartX+4){
            schedule(intakeSys.runIntake(IntakeSys.intakeInPower));
            schedule(intakeSys.setStack1(IntakeSys.intakeServoLowPosition));
            schedule(intakeSys.setStack2(IntakeSys.intakeServo2LowPosition));
            currentState = State.WAIT_FOR_FINISH;
        }
        if(currentState == State.WAIT_FOR_FINISH && !drive.isBusy()){
            //TODO: should it eject here?
            schedule(intakeSys.runIntake(-IntakeSys.intakeOutPower));
            schedule(new DelayedCommand(intakeSys.runIntake(0), 1000));
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
