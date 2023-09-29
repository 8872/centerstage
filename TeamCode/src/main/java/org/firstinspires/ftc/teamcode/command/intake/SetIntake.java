package org.firstinspires.ftc.teamcode.command.intake;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSys;

public class SetIntake extends CommandBase {
    private final IntakeSys sys;
    private final IntakeSys.IntakeDirection direction;
    private final IntakeSys.StackHeight height;
    public SetIntake(IntakeSys sys, IntakeSys.IntakeDirection direction, IntakeSys.StackHeight height) {
        this.sys = sys;
        this.direction = direction;
        this.height = height;
        addRequirements(sys);
    }
    @Override
    public void initialize() {
        sys.intake(direction);
        sys.stackHeight(height);
    }

    /**
     * Implement something so it automatically finishes once color sensor detects something less than 10cm away(basically inside of intake)
     * @return
     */
    @Override
    public boolean isFinished() {
        return true;
    }
}
