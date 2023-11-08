package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystem.*;

import java.math.BigDecimal;
import java.math.RoundingMode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.*;

@Config
public class DriveBaseOpMode extends CommandOpMode {
    //working
    public static double axonServoHome = 0.8;
    public static double axonServoAway = 0.35;

    public static double pitchServoHome = 0.43;
    public static double pitchServoAway = 1;
    protected MotorEx fL, fR, bL, bR, intakeMotor, lil, lir;
    public ColorSensor colorSensor;
    protected SimpleServo stackServo, launcherHeightServo, launcherServo, pitchServo, clawL,clawR, armServo;
    protected DriveSys drive;
    protected LiftSubsystem liftSys;
    protected IntakeSubsystem intakeSys;
    protected GamepadEx gamepadEx1;
    protected GamepadEx gamepadEx2;
    protected LauncherSys launcherSys;
    protected ArmSys armSys;
    protected ClawSys clawSys;
    protected IMU imu;
    protected TriggerReader rightTriggerReader, leftTriggerReader;
    @Override
    public void initialize() {
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        rightTriggerReader = new TriggerReader(gamepadEx1, RIGHT_TRIGGER);
        leftTriggerReader = new TriggerReader(gamepadEx1, LEFT_TRIGGER);
        initHardware();
        setUpHardwareDevices();
        drive = new DriveSys(fL, fR, bL, bR, imu);
        intakeSys = new IntakeSubsystem(intakeMotor, stackServo, colorSensor, () -> gamepadEx1.gamepad.right_trigger, () -> gamepadEx1.gamepad.left_trigger);
        launcherSys = new LauncherSys(launcherHeightServo, launcherServo);
        armSys = new ArmSys(pitchServo, armServo);
        liftSys = new LiftSubsystem(lil, lir, () -> gamepadEx1.gamepad.touchpad_finger_1_x);
        clawSys = new ClawSys(clawL,clawR);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Mode", "Done initializing");
        telemetry.update();
    }
    protected void initHardware() {
        stackServo = new SimpleServo(hardwareMap, "stackServo", 0 ,255);
        launcherHeightServo = new SimpleServo(hardwareMap, "launcherServo", 0,255);
        launcherServo = new SimpleServo(hardwareMap, "launch", 0,255);
        pitchServo = new SimpleServo(hardwareMap, "pitch",0,270);
        clawL = new SimpleServo(hardwareMap, "clawL", 0,255);
        clawR = new SimpleServo(hardwareMap, "clawR", 0,255);
        armServo = new SimpleServo(hardwareMap, "axon", 0,255);
        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        if (colorSensor instanceof SwitchableLight) ((SwitchableLight)colorSensor).enableLight(true);
        fL = new MotorEx(hardwareMap, "fl");
        fR = new MotorEx(hardwareMap, "fr");
        bL = new MotorEx(hardwareMap, "bl");
        bR = new MotorEx(hardwareMap, "br");
        intakeMotor = new MotorEx(hardwareMap, "intakeMotor");
        lil = new MotorEx(hardwareMap, "lil");
        lir = new MotorEx(hardwareMap, "lir");
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.DOWN)));
    }
    @Override
    public void run() {
        super.run();
        tad("leftFront Power", round(fL.motor.getPower()));
        tad("leftBack Power", round(bL.motor.getPower()));
        tad("rightFront Power", round(fR.motor.getPower()));
        tad("rightBack Power", round(bR.motor.getPower()));
        tad("intakeMotor power", round(intakeMotor.motorEx.getCurrent(CurrentUnit.AMPS)));
        tad("lil pos", lil.getCurrentPosition());
        tad("lir pos", lir.getCurrentPosition());
        tad("intakeServoPos", round(stackServo.getPosition()));
        telemetry.update();
    }

    protected void setUpHardwareDevices() {
        lir.setInverted(true);
        intakeMotor.setInverted(true);
        lil.stopAndResetEncoder();
        lir.stopAndResetEncoder();
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

    protected GamepadButton gb2(GamepadKeys.Button button){
        return gamepadEx2.getGamepadButton(button);
    }



    // telemetry add data = tad
    protected void tad(String caption, Object value){
        telemetry.addData(caption, value);
    }
}
