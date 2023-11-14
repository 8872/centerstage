import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.util.commands.CycleCommand;
import org.junit.Test;

public class CommandTest {
    @Test
    public void cycle() {
        CycleCommand a = new CycleCommand(
                new InstantCommand(() -> System.out.println("amogus")),
                new InstantCommand(() -> System.out.println("sus")),
                new InstantCommand(() -> System.out.println("baka"))
        );
        CommandScheduler.getInstance().schedule(a);
        CommandScheduler.getInstance().run();
        CommandScheduler.getInstance().run();
        CommandScheduler.getInstance().run();
        CommandScheduler.getInstance().run();
        CommandScheduler.getInstance().run();
        CommandScheduler.getInstance().run();
        CommandScheduler.getInstance().run();

    }
}
