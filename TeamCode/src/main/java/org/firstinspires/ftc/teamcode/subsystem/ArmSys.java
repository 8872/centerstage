package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
@Config
public class ArmSys {
    public enum ArmPosition {
        HOME(0.0),
        AWAY(1.0);
        public final double pos;
        ArmPosition(double pos) {
            this.pos = pos;
        }
    }
    private final ServoEx armL;
    private final ServoEx armR;
    public ArmSys(ServoEx armL, ServoEx armR) {
        this.armL = armL;
        this.armR = armR;
    }

    public void arm(ArmPosition position) {
        armL.setPosition(position.pos);
        armR.setPosition(position.pos);
    }

    public void arm(double position) {
        armL.setPosition(position);
        armR.setPosition(position);
    }
}
