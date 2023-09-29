package org.firstinspires.ftc.teamcode.command.intake;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSys;

public class SetIntakeDirection extends CommandBase {
    public final IntakeSys sys;
    public final IntakeSys.IntakeDirection direction;
    public SetIntakeDirection(IntakeSys sys, IntakeSys.IntakeDirection direction) {
        this.sys = sys;
        this.direction = direction;
        addRequirements(sys);
    }
    @Override
    public void initialize() {
        sys.intake(direction);
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
