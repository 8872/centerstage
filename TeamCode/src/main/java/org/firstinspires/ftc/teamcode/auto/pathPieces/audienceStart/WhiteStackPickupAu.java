package org.firstinspires.ftc.teamcode.auto.pathPieces.audienceStart;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.auto.util.AutoBaseOpmode;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSys;
import org.firstinspires.ftc.teamcode.util.commands.DelayedCommand;

@Config
@TeleOp(name="White Stack Au", group = "ZZZ")
public class WhiteStackPickupAu extends AutoBaseOpmode {

    public enum State{
        WAIT_FOR_START,
        TOPPLE_STACK1,
        TOPPLE_STACK2,
        PICKUP_PIXELS,
        WAIT_FOR_INTAKE,
        EJECT,
        WAIT_FOR_EJECT,
        FINISHED
    }
    public static State currentState = State.WAIT_FOR_START;

    public static boolean red = true;

    public static double intakeInitialPower = 0;
    public static double intakeKnockPower = 0.4;
    public static double intakeFinalPower = 0.67;
    public static double pixelX = -56;
    public static double pixelY = 36;
    public static double moveForwardDistance = 6;
    public static double moveBackDistance = 2;
    public static double pickUpPathSpeed = 20;
    public static double moveBackwardsSpeed = 10;
    public static double initialForwardSpeed = 15;
    public static double intakeWaitTime = 500;

    public static boolean testing = false;

    private ElapsedTime intakeWaitTimer;

    public WhiteStackPickupAu(){
        super.init();
        setUp();
    }

    @Override
    public void init(){
        super.init();
        intakeWaitTimer = new ElapsedTime();
        testing = false;
        if(red)
            drive.setPoseEstimate(new Pose2d(pixelX, -pixelY, Math.toRadians(180)));
        else
            drive.setPoseEstimate(new Pose2d(pixelX, pixelY, Math.toRadians(180)));
    }

    public void setUp(){
        intakeWaitTimer = new ElapsedTime();
    }

    @Override
    public void loop() {
        super.loop();
        drive.update();

        if (gamepad1.right_bumper && !testing) {
            testing = true;
            currentState = State.TOPPLE_STACK1;
        }
        if (currentState == State.TOPPLE_STACK1) {
            schedule(armSys.intake());
            schedule(intakeSys.setStack1(IntakeSys.intakeServoHighPosition));
            schedule(intakeSys.setStack2(IntakeSys.intakeServoHighPosition));
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
            currentState = State.TOPPLE_STACK2;
        }
        if(currentState == State.TOPPLE_STACK2 && !drive.isBusy()){
            schedule(intakeSys.setStack1(IntakeSys.intakeServoLowPosition));
            schedule(intakeSys.setStack2(IntakeSys.intakeServoLowPosition));
            schedule(intakeSys.runIntake(intakeKnockPower));
            if (red) {
                drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(pixelX, -pixelY, Math.toRadians(180.00)))
                        .lineToLinearHeading(new Pose2d(pixelX + moveBackDistance, -pixelY, Math.toRadians(180.00)),
                                SampleMecanumDrive.getVelocityConstraint(moveBackwardsSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .forward(2)
                        .waitSeconds(0.5)
                        .build());
            } else {
                drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(pixelX, pixelY, Math.toRadians(180.00)))
                        .lineToLinearHeading(new Pose2d(pixelX + moveBackDistance, pixelY, Math.toRadians(180.00)),
                                SampleMecanumDrive.getVelocityConstraint(moveBackwardsSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .forward(2)
                        .waitSeconds(0.5)
                        .build());
            }
            currentState = State.PICKUP_PIXELS;
        }
        if (currentState == State.PICKUP_PIXELS && drive.getPoseEstimate().getX() > pixelX) {
            schedule(intakeSys.runIntake(intakeFinalPower));
            currentState = State.WAIT_FOR_INTAKE;
        }
        if (currentState == State.WAIT_FOR_INTAKE && !drive.isBusy()) {
            intakeWaitTimer.reset();
            currentState = State.EJECT;
        }
        if (currentState == State.EJECT && intakeWaitTimer.milliseconds()>intakeWaitTime) {
            schedule(intakeSys.setStack1(IntakeSys.intakeServoHighPosition));
            schedule(intakeSys.setStack2(IntakeSys.intakeServoHighPosition));
            schedule(intakeSys.runIntake(-IntakeSys.intakeOutPower));
            schedule(new DelayedCommand(intakeSys.runIntake(0), 1000));
        }
        if (currentState == State.WAIT_FOR_EJECT && intakeSys.getIntakePower() == 0) {
            currentState = State.FINISHED;
            testing = false;
        }
        if (currentState != State.FINISHED){ //TODO: add pixel counter condition
            //set state to finished once pixels == 2
        }
    }
    public void run(boolean redSide){
        red = redSide;
        currentState = State.TOPPLE_STACK1;
    }
    public boolean isFinished(){
        return currentState == State.FINISHED;
    }
}
