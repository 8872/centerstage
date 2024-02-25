package org.firstinspires.ftc.teamcode.opmode.tests;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Motor Test Opmode", group = "Tuner")
@Disabled
public class MotorTestOpmode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MotorEx leftRear = new MotorEx(hardwareMap, "leftFront", Motor.GoBILDA.RPM_435);
        MotorEx rightFront = new MotorEx(hardwareMap, "leftRear", Motor.GoBILDA.RPM_435);
        MotorEx rightRear = new MotorEx(hardwareMap, "rightFront", Motor.GoBILDA.RPM_435);
        MotorEx leftFront = new MotorEx(hardwareMap, "rightRear", Motor.GoBILDA.RPM_435);

        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            if(gamepad1.a){
                leftFront.set(0.5);
            }if(gamepad1.b){
                leftRear.set(0.5);
            }if(gamepad1.x){
                rightFront.set(0.5);
            }if(gamepad1.y){
                rightRear.set(0.5);
            }
        }
    }
}
