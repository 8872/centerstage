package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

@TeleOp(name = "Lift Auto", group = "Testing")
public class LiftTestAuto extends DriveBaseOpMode{

    public static double BOTTOM = 0;
    public static double TOP = -570;
    public double LIFT_STATE = 0;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void initialize() {
        super.initialize();
        gb1(GamepadKeys.Button.Y).whenPressed(liftSys.goTo(LiftSubsystem.THIRD));
        gb1(GamepadKeys.Button.B).whenPressed(liftSys.goTo(LiftSubsystem.SECOND));
        gb1(GamepadKeys.Button.X).whenPressed(liftSys.goTo(LiftSubsystem.FIRST));
        gb1(GamepadKeys.Button.A).whenPressed(liftSys.goTo(LiftSubsystem.NONE));
        register(drive, intakeSys, armSubsystem, boxSubsystem, liftSys, launcherSubsystem);
        drive.setDefaultCommand(drive.drive(gamepadEx1::getLeftX,gamepadEx1::getLeftY,gamepadEx1::getRightX));
    }
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        timer.reset();
        while (!isStopRequested() && opModeIsActive()) {
            run();
        }
        reset();
    }
    @Override
    public void run() {
        super.run();
        tad("lil pos", lil.getCurrentPosition());
        tad("lir pos", lir.getCurrentPosition());
        tad("lil vel", lil.getVelocity());
        tad("lir vel", lir.getVelocity());
        tad("Target Position", liftSys.getTargetHeight());
        tad("Lift State", LIFT_STATE);
        tad("ElapsedTime", timer);
        telemetry.update();

        if(timer.seconds() > 5 && LIFT_STATE == 0){
            liftSys.goTo(BOTTOM);
            timer.reset();
            LIFT_STATE = 1;
        }
        if(timer.seconds() > 5 && LIFT_STATE == 1){
            liftSys.goTo(TOP);
            timer.reset();
            LIFT_STATE = 0;
        }
    }

}