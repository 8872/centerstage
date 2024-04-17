package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Config
@TeleOp(name = "Motor Test Opmode", group = "Tuner")

public class MotorTestOpmode extends LinearOpMode {
    public static double power = 0.3;
    @Override
    public void runOpMode() throws InterruptedException {
        MotorEx lil = new MotorEx(hardwareMap, "lil", Motor.GoBILDA.RPM_435);
        MotorEx lir = new MotorEx(hardwareMap, "lir", Motor.GoBILDA.RPM_435);


        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            if(gamepad1.a){
                lil.set(power);
            } else {
                lil.set(0);
            }
            }if(gamepad1.b){
                lir.set(power);
            } else {
                lir.set(0);
            }
    }
}
