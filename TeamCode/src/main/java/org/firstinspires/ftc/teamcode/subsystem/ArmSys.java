package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
@Config
public class ArmSys extends SubsystemBase {
    public enum ArmPosition {
        HOME(0.0),
        AWAY(1.0);
        public final double pos;
        ArmPosition(double pos) {
            this.pos = pos;
        }
    }
    public ArmPosition armPos;
    private final ServoEx armL;
    private final ServoEx armR;
    public ArmSys(ServoEx armL, ServoEx armR) {
        this.armL = armL;
        this.armR = armR;
        armPos = ArmPosition.HOME;
    }

    public void arm(ArmPosition position) {
        armL.setPosition(position.pos);
        armR.setPosition(position.pos);
        armPos = position;
    }

    public void arm(double position) {
        armL.setPosition(position);
        armR.setPosition(position);
    }
    public double getArmPos() {
        return armL.getPosition();
    }
    public ArmPosition getPos() {
        return armPos;
    }

}
