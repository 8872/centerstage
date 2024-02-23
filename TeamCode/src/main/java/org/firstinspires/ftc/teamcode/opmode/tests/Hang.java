package org.firstinspires.ftc.teamcode.opmode.tests;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Hang extends OpMode {
    private MotorEx hang;

    @Override
    public void init() {
        hang = new MotorEx(hardwareMap, "hang", Motor.GoBILDA.RPM_30);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            hang.set(1);
        } else if (gamepad1.b) {
            hang.set(-1);
        }
        hang.set(0);

        telemetry.addData("hang power", hang.get());
        telemetry.update();
    }
}
