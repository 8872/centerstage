package org.firstinspires.ftc.teamcode.auto.pathPieces.backdropStart;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.auto.util.AutoBaseOpmode;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSys;
import org.firstinspires.ftc.teamcode.util.commands.DelayedCommand;

@Config
@Disabled
@TeleOp(name="White Stack Ba", group = "ZZZ")
public class WhiteStackPickupBa extends AutoBaseOpmode {

    public enum State{
        WAIT_FOR_START,
        TOPPLE_STACK,
        PICKUP_PIXELS,
        EJECT,
        WAIT_FOR_EJECT,
        FINISHED
    }
    public static State currentState = State.WAIT_FOR_START;

    public static boolean red = true;

    public static double intakeInitialPower = 0.6;
    public static double intakeFinalPower = 0.8;
    public static double pixelX = -56;
    public static double pixelY = 36;
    public static double moveForwardDistance = 4;
    public static double moveBackDistance = 4;

    public static boolean testing = false;

    @Override
    public void init(){
        if(red)
            drive.setPoseEstimate(new Pose2d(pixelX, -pixelY, Math.toRadians(180)));
        else
            drive.setPoseEstimate(new Pose2d(pixelX, pixelY, Math.toRadians(180)));
    }

    @Override
    public void loop() {
        super.loop();

        if (gamepad1.right_bumper && !testing) {
            testing = true;
            currentState = State.TOPPLE_STACK;
        }
        if (currentState == State.TOPPLE_STACK) {
            schedule(intakeSys.setStack1(IntakeSys.intakeServoLowPosition));
            schedule(intakeSys.setStack2(IntakeSys.intakeServo2LowPosition));
            schedule(intakeSys.runIntake(intakeInitialPower));
            if (red) {
                drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(pixelX, -pixelY, Math.toRadians(180.00)))
                        .lineToLinearHeading(new Pose2d(pixelX - moveForwardDistance, -pixelY, Math.toRadians(180.00)))
                        .lineToLinearHeading(new Pose2d(pixelX + moveBackDistance, -pixelY, Math.toRadians(180.00)))
                        .lineToLinearHeading(new Pose2d(pixelX, -pixelY, Math.toRadians(180.00)))
                        .waitSeconds(0.5)
                        .build());
            } else {
                drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(pixelX, pixelY, Math.toRadians(180.00)))
                        .lineToLinearHeading(new Pose2d(pixelX - moveForwardDistance, pixelY, Math.toRadians(180.00)))
                        .lineToLinearHeading(new Pose2d(pixelX + moveBackDistance, pixelY, Math.toRadians(180.00)))
                        .lineToLinearHeading(new Pose2d(pixelX, pixelY, Math.toRadians(180.00)))
                        .waitSeconds(0.5)
                        .build());
            }
            currentState = State.PICKUP_PIXELS;
        }
        if (currentState == State.PICKUP_PIXELS && drive.getPoseEstimate().getX() > pixelX + 3.5) {
            schedule(intakeSys.runIntake(intakeFinalPower));
            currentState = State.EJECT;
        }
        if (currentState == State.EJECT && !drive.isBusy()) {
            schedule(intakeSys.setStack1(IntakeSys.intakeServoHighPosition));
            schedule(intakeSys.setStack2(IntakeSys.intakeServo2HighPosition));
            schedule(intakeSys.runIntake(-IntakeSys.intakeOutPower));
            schedule(new DelayedCommand(intakeSys.runIntake(0), 1000));
            currentState = State.WAIT_FOR_EJECT;
        }
        if (currentState == State.WAIT_FOR_EJECT) {
            if (intakeSys.getIntakePower() == 0) {
                //TODO: Change it so it loops until 2 pixels are detected
                //currentState = State.TOPPLE_STACK;
                currentState = State.FINISHED;
                testing = false;
            }
        }
        if (currentState != State.FINISHED){ //TODO: add pixel counter condition
            //set state to finished once pixels == 2
        }
    }
    public static void run(boolean redSide){
        red = redSide;
        currentState = State.TOPPLE_STACK;
    }
    public static boolean isFinished(){
        return currentState == State.FINISHED;
    }
}