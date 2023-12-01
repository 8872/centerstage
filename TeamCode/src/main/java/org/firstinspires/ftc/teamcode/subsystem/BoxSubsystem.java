package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.util.commands.CycleCommand;
@Config
public class BoxSubsystem extends SubsystemBase {
    public static double STOP_FIRST = 0.5;
    public static double STOP_SECOND = 0.7;
    public static double RELEASE_FIRST = 0.7;
    public static double RELEASE_SECOND = 0.55;

    private Gamepad gamepad;
    private final ServoEx first;
    private final ServoEx second;
    public enum ClawState {
        FIRST,
        SECOND
    }

    public ClawState clawState;
    public BoxSubsystem(ServoEx first, ServoEx second, Gamepad gamepad) {
        this.first = first;
        this.second = second;
    }

    public Command cycle() {
        return new CycleCommand(
                new InstantCommand(() -> {
                    first.setPosition(STOP_FIRST);
                    second.setPosition(STOP_SECOND);
                }),
                new InstantCommand(() -> first.setPosition(RELEASE_FIRST)),
                new InstantCommand(() -> second.setPosition(RELEASE_SECOND))
        );
    }

    public void cycleCommand() {
        switch (clawState) {
            case FIRST:
                clawState = ClawState.SECOND;
                first.setPosition(RELEASE_FIRST);
                second.setPosition(RELEASE_SECOND);
                break;
            case SECOND:
                first.setPosition(RELEASE_FIRST);
                second.setPosition(RELEASE_SECOND);
                break;
        }
    }

    public Command depositNext(){
        return new InstantCommand(() -> {
            if(secondClosed()) second.setPosition(RELEASE_SECOND);
            else first.setPosition(RELEASE_FIRST);
        });
    }

    public boolean secondClosed(){
        return getSecondPosition() > (STOP_SECOND-0.01);
    }

    public Command stopFirst() {
        return new InstantCommand(() -> first.setPosition(STOP_FIRST));
    }

    public Command releaseFirst() {
        return new InstantCommand(() -> first.setPosition(RELEASE_FIRST));
    }

    public Command stopSecond() {
        return new InstantCommand(() -> second.setPosition(STOP_SECOND));
    }

    public Command releaseSecond() {
        return new InstantCommand(() -> second.setPosition(RELEASE_SECOND));
    }

    public double getFirstPosition() {
        return first.getPosition();
    }

    public double getSecondPosition() {
        return second.getPosition();
    }

    public void resetBoxState(){
        clawState = ClawState.FIRST;
    }
}
