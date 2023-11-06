package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
@Config
public class ClawSys extends SubsystemBase {
    /**
     * 1 servo
     */
    public SimpleServo servoL, servoR;

    public static double servoLRelease=0.5;
    public static double servoLHold=1;

    public static double servoRRelease =0.5;
    public static double servoRHold =1;

    public ClawSys(SimpleServo servoL, SimpleServo servoR) {
        this.servoL = servoL;
        this.servoR = servoR;
    }
    public Command holdFirst() {return new InstantCommand(()-> servoL.setPosition(servoLHold));}
    public Command releaseFirst() {return new InstantCommand(() -> servoL.setPosition(servoLRelease));}

    public Command holdSecond() {return new InstantCommand(()-> servoR.setPosition(servoRHold));}
    public Command releaseSecond() {return new InstantCommand(()->servoR.setPosition(servoRRelease));}

}
