//package org.firstinspires.ftc.teamcode.command.arm;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//import org.firstinspires.ftc.teamcode.subsystem.ArmSys;
//
//public class SetArm extends CommandBase {
//    private final ArmSys sys;
//    private final ArmSys.ArmPosition position;
//    public SetArm(ArmSys sys, ArmSys.ArmPosition position) {
//        this.sys = sys;
//        this.position = position;
//        addRequirements(sys);
//    }
//    @Override
//    public void initialize() {
//        sys.arm(position);
//    }
//    @Override
//    public boolean isFinished() {
//        return sys.getPos() == position;
//    }
//}
