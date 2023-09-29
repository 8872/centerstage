package org.firstinspires.ftc.teamcode.command.claw;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ProfiledPIDCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.TrapezoidProfileCommand;
import com.arcrobotics.ftclib.controller.PIDFController;
import org.firstinspires.ftc.teamcode.subsystem.ClawSys;

public class SetClaw extends CommandBase {
    private final ClawSys sys;
    private final ClawSys.ClawPosition position;
    public SetClaw(ClawSys sys, ClawSys.ClawPosition position) {
        this.sys = sys;
        this.position = position;
        addRequirements(sys);
    }
    @Override
    public void initialize() {
        sys.claw(position);
    }
    @Override
    public boolean isFinished() {
        return sys.getClaw() == position;
    }

}
