package org.firstinspires.ftc.teamcode.opmode.tests;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class Stack2 extends OpMode {
    SimpleServo stack, stack2;
    @Override
    public void init() {
        stack = new SimpleServo(hardwareMap, "stack", 0, 255);
        stack2 = new SimpleServo(hardwareMap, "stack2", 0, 255);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            stack.setPosition(1);
            stack2.setPosition(1);
        } else if (gamepad1.b) {
            stack.setPosition(0);
            stack2.setPosition(0);
        }

        telemetry.addData("pos", stack.getPosition());
        telemetry.update();
    }
}
