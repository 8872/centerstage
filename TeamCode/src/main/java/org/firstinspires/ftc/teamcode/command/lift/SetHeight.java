package org.firstinspires.ftc.teamcode.command.lift;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.LiftSys;

public class SetHeight extends CommandBase {
    private final LiftSys sys;
    private final LiftSys.Height height;
    public SetHeight(LiftSys sys, LiftSys.Height height) {
        this.sys = sys;
        this.height = height;
        addRequirements(sys);
    }
    @Override
    public void initialize() {
        sys.setHeight(height);
    }
    @Override
    public boolean isFinished() {
        return sys.atTarget();
    }
}
