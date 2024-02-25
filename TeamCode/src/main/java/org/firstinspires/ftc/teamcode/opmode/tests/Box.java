package org.firstinspires.ftc.teamcode.opmode.tests;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;

@TeleOp
@Disabled
public class Box extends BaseOpMode {
    @Override
    public void initialize() {
        super.initialize();

        gb1(GamepadKeys.Button.A).whenPressed(new InstantCommand(() -> boxSys.release()));
        gb1(GamepadKeys.Button.B).whenPressed(boxSys.close());
    }

    @Override
    public void run() {
        super.run();

        tad("inner pos", boxSys.getInnerServo());
        tad("outer pos", boxSys.getOuterServo());
        telemetry.update();
    }
}
