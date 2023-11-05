package org.firstinspires.ftc.teamcode.util.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.WaitCommand;

public class DelayedCommand extends SequentialCommandGroup {
    // delay is in milliseconds
    public DelayedCommand(Command command, long delay){
        addCommands(
                new WaitCommand(delay),
                command
        );

        addRequirements(command.getRequirements().toArray(new Subsystem[0]));
    }
}