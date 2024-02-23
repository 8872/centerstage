package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;

@Config
public class PlaneSys extends SubsystemBase {
    private final SimpleServo servo;
    public static double releasePos = 0.5;

    public PlaneSys(SimpleServo servo) {
        this.servo = servo;
    }

    public Command launch() {
        return new InstantCommand(() -> servo.setPosition(releasePos));
    }

}
