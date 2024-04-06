package org.firstinspires.ftc.teamcode.auto.pathPieces.audienceStart;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.auto.util.AutoBaseOpmode;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.util.commands.DelayedCommand;

@Config
@TeleOp(name="To Stacks Au", group = "ZZZ")
public class BackdropToStacksAu extends AutoBaseOpmode {

    public enum ToStackState {
        WAIT_FOR_START,
        MOVE_TO_STACKS,
        RUN_INTAKE,
        WAIT_FOR_FINISH,
        FINISHED
    }
    public static ToStackState currentToStackState = ToStackState.WAIT_FOR_START;

    public static boolean red = true;

    public static double pixelPathSpeed = 10;
    public static double toBottomRowX = 30;
    public static double toBottomPathY = 52;
    public static double returnLineToX = -26;
    public static double returnLineToY = 59;
    public static double pixelPathStartX = -48;
    public static double pixelPathY = 36;
    public static double pixelPathEndX = -59;

    public static boolean testing = false;

    @Override
    public void init(){
        super.init();
        //TODO: remove setPosEstimate
        if(red)
            drive.setPoseEstimate(new Pose2d(50, -44, Math.toRadians(180)));
        else
            drive.setPoseEstimate(new Pose2d(50, 44, Math.toRadians(180)));
    }

    @Override
    public void loop() {
        super.loop();
        drive.update();

        if(gamepad1.right_bumper && !testing){
            testing = true;
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
            currentToStackState = ToStackState.WAIT_FOR_FINISH;
        }
        if(currentToStackState == ToStackState.WAIT_FOR_FINISH && !drive.isBusy()){
            //TODO: should it eject here?
            schedule(intakeSubsystem.runIntake(-IntakeSubsystem.intakeOutPower));
            schedule(new DelayedCommand(intakeSubsystem.runIntake(0), 1000));
            currentToStackState = ToStackState.FINISHED;
        }
    }

    public static void run(boolean redSide){
        red = redSide;
        currentToStackState = ToStackState.MOVE_TO_STACKS;
    }
    public static boolean isFinished(){
        return currentToStackState == ToStackState.FINISHED;
    }
}
