package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.HighestSampleFilter;
import org.firstinspires.ftc.teamcode.util.MB1242;
import org.firstinspires.ftc.teamcode.util.wpilib.LinearFilter;
import org.firstinspires.ftc.teamcode.util.wpilib.MedianFilter;

@TeleOp
@Config
public class Distance extends OpMode {
    private MB1242 fl;
    private MB1242 fr;
    private MB1242 bl;
    public static int highestSize = 20;
    public static int medianSize = 20;
    private HighestSampleFilter filter = new HighestSampleFilter(highestSize);

    @Override
    public void init() {
        fl = hardwareMap.get(MB1242.class, "flSensor");
        fr = hardwareMap.get(MB1242.class, "frSensor");
        bl = hardwareMap.get(MB1242.class, "blSensor");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        telemetry.addData("fl", fl.getDistance(DistanceUnit.INCH));
        telemetry.addData("fr", fr.getDistance(DistanceUnit.INCH));
        telemetry.addData("bl", bl.getDistance(DistanceUnit.INCH));
        telemetry.addData("bl filtered", filter.calculate(bl.getDistance(DistanceUnit.INCH)));
        telemetry.update();
    }
}
