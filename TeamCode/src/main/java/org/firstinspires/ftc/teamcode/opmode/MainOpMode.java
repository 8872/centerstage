package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Main Op Mode")
public class MainOpMode extends BaseOpMode {
    @Override
    public void initialize() {
        super.initialize();

        gb1(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(boxSys.release());
        gb1(GamepadKeys.Button.LEFT_BUMPER).toggleWhenPressed(new ParallelCommandGroup(armSys.deposit(), boxSys.close()), new ParallelCommandGroup(armSys.intake(), boxSys.intake()));
    }
}
