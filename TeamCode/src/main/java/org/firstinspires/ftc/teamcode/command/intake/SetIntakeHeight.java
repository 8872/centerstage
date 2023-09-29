package org.firstinspires.ftc.teamcode.command.intake;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSys;

public class SetIntakeHeight extends CommandBase {
    private final IntakeSys sys;
    private final IntakeSys.StackHeight position;
    public SetIntakeHeight(IntakeSys sys, IntakeSys.StackHeight position) {
        this.sys = sys;
        this.position = position;
        addRequirements(sys);
    }
    @Override
    public void initialize() {
        sys.stackHeight(position);
    }
    @Override
    public boolean isFinished() {
        return sys.getStackHeightPos() == position.pos;
    }
}
