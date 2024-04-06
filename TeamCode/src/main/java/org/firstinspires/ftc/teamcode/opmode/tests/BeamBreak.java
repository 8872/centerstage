package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.util.wpilib.MedianFilter;

@TeleOp
@Config
@Disabled
public class BeamBreak extends BaseOpMode {
    public static int sampleSize = 15;
    private final MedianFilter filter = new MedianFilter(sampleSize);

    private int pixelCount = 0;

    public static double FIRST = 123;
    public static double SECOND = 105;
    public static double tolerance = 4;

    @Override
    public void initialize() {
        super.initialize();

        gb1(GamepadKeys.Button.A).whenPressed(new InstantCommand(filter::reset));

        intakeSubsystem.setDefaultCommand(intakeSubsystem.intake(
                () -> gamepadEx1.gamepad.right_trigger,
                () -> gamepadEx1.gamepad.left_trigger)
        );
    }

    @Override
    public void run() {
        super.run();

        double filtered = filter.calculate(beam2.getDistance(DistanceUnit.MM));
        if (filtered < (FIRST + tolerance) && filtered > (FIRST - tolerance)) {
            pixelCount = 1;
        } else if (filtered < (SECOND + tolerance) && filtered > (SECOND - tolerance)) {
            pixelCount = 2;
        }
        telemetry.addData("pixel count:", pixelCount);
        telemetry.addData("filtered mm:", filtered);
        telemetry.update();
    }
}
