package org.firstinspires.ftc.teamcode.util.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Subsystem;

import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

public class CycleCommand implements Command {
    private final List<Command> commands;
    private int count = 1;

    public CycleCommand(Command... commands) {
        this.commands = Arrays.asList(commands);
    }

    @Override
    public void execute() {
        CommandScheduler.getInstance().schedule(commands.get(count - 1));
        if (count % commands.size() == 0) {
            count = 1;
        } else {
            count++;
        }
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return commands.stream()
                .flatMap(command -> command.getRequirements().stream())
                .collect(Collectors.toSet());
    }
}
