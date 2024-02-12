package org.firstinspires.ftc.teamcode.opmode.tests;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystem.PlaneSys;
@TeleOp(name="Plane Servo Tuner", group="Tuner")
public class PlaneServoTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SimpleServo plane = new SimpleServo(hardwareMap, "airplane", 0,255);
        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            plane.setPosition(PlaneSys.releasePos);
        }
    }
}
