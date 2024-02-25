package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;

@Config
@TeleOp
@Disabled
public class LiftMotorTestOpmode extends BaseOpMode {

    public static double powerRight = -0.1;
    public static double powerLeft = -0.1;

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void run(){
        super.run();
        if(gamepad1.x){
            liftLeft.set(powerLeft);
        }
        if(gamepad1.y){
            liftRight.set(powerRight);
        }
    }
}
