package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystem.DriveSys;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSys;
import org.firstinspires.ftc.teamcode.subsystem.LiftSys;

import java.math.BigDecimal;
import java.math.RoundingMode;

public class DriveBaseOpMode extends CommandOpMode {
    protected MotorEx fL, fR, bL, bR, intakeMotor, lil, lir;
    protected SimpleServo stackServo;
    protected DriveSys drive;
    protected LiftSys liftSys;
    protected IntakeSys intakeSys;
    protected GamepadEx gamepadEx1;
    protected RevIMU imu;
    @Override
    public void initialize() {
        gamepadEx1 = new GamepadEx(gamepad1);
        initHardware();
        setUpHardwareDevices();
        drive = new DriveSys(fL, fR, bL, bR);
        intakeSys = new IntakeSys(intakeMotor, stackServo);
        liftSys = new LiftSys(lil, lir, gamepadEx1::getRightY);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Mode", "Done initializing");
        telemetry.update();
    }
    protected void initHardware() {
        stackServo = new SimpleServo(hardwareMap, "stackServo", 0 ,255);
        fL = new MotorEx(hardwareMap, "fl");
        fR = new MotorEx(hardwareMap, "fr");
        bL = new MotorEx(hardwareMap, "bl");
        bR = new MotorEx(hardwareMap, "br");
        intakeMotor = new MotorEx(hardwareMap, "intakeMotor");
        lil = new MotorEx(hardwareMap, "lil");
        lir = new MotorEx(hardwareMap, "lir");
    }
    @Override
    public void run() {
        super.run();
        tad("leftFront Power", round(fL.motor.getPower()));
        tad("leftBack Power", round(bL.motor.getPower()));
        tad("rightFront Power", round(fR.motor.getPower()));
        tad("rightBack Power", round(bR.motor.getPower()));
        tad("intakeMotor power", round(intakeMotor.motorEx.getCurrent(CurrentUnit.AMPS)));
        telemetry.update();
    }

    protected void setUpHardwareDevices() {
        lir.setInverted(true);
        intakeMotor.setInverted(true);
    }


    private static double round(double value, @SuppressWarnings("SameParameterValue") int places) {
        if (places < 0) throw new IllegalArgumentException();

        return new BigDecimal(Double.toString(value)).setScale(places, RoundingMode.HALF_UP).doubleValue();
    }

    private static double round(double value) {
        return round(value, 4);
    }

    // gamepad button 1 = gb1
    protected GamepadButton gb1(GamepadKeys.Button button){
        return gamepadEx1.getGamepadButton(button);
    }



    // telemetry add data = tad
    protected void tad(String caption, Object value){
        telemetry.addData(caption, value);
    }
}
