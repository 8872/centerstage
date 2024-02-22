package org.firstinspires.ftc.teamcode.opmode.tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class BeamBreak extends CommandOpMode {
    private DistanceSensor beam2;

    @Override
    public void initialize() {
        beam2 = hardwareMap.get(DistanceSensor.class, "beam2");
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("cm:", beam2.getDistance(DistanceUnit.CM));
        telemetry.addData("mm:", beam2.getDistance(DistanceUnit.MM));
        telemetry.update();
    }
}
