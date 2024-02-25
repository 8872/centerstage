package org.firstinspires.ftc.teamcode.opmode.tests;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Motor Position Tuner", group = "Tuner")
@Disabled
public class MotorPosTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MotorEx rightLift = new MotorEx(hardwareMap, "lir", Motor.GoBILDA.RPM_1150);
        rightLift.setInverted(true);
        MotorEx leftLift = new MotorEx(hardwareMap,"lil", Motor.GoBILDA.RPM_1150);
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            telemetry.addData("Right Position", rightLift.getCurrentPosition());
            telemetry.addData("Left Position", leftLift.getCurrentPosition());
            telemetry.update();
        }
    }
}
