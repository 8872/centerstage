package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;

@Config
public class PlaneSys extends SubsystemBase {
    private final SimpleServo servo;
    public static double staticPos = 0.2;
    public static double releasePos = 0.7;

    public PlaneSys(SimpleServo servo) {
        this.servo = servo;
        servo.setPosition(staticPos);
    }

    public Command launch() {
        return new InstantCommand(() -> servo.setPosition(releasePos));
    }

}
