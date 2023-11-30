package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystem.*;
import org.firstinspires.ftc.teamcode.util.booster.PhotonCore;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;


import android.graphics.Bitmap;
import android.graphics.Canvas;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.concurrent.atomic.AtomicReference;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;

import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.concurrent.atomic.AtomicReference;

@Config
public class DriveBaseOpMode extends CommandOpMode {

    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) {
            // do nothing
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }
    }

    protected MotorEx fL, fR, bL, bR, intakeMotor, lil, lir;
    protected SimpleServo stackServo, launcherHeightServo, launcherServo, pitchServo, clawL,clawR, armServo;
    protected TouchSensor limitSwitchL, limitSwitchR;
    protected DriveSubsystem drive;
    protected LiftSubsystem liftSys;
    protected IntakeSubsystem intakeSys;
    protected GamepadEx gamepadEx1;
    protected GamepadEx gamepadEx2;
    protected LauncherSubsystem launcherSubsystem;
    protected ArmSubsystem armSubsystem;
    protected BoxSubsystem boxSubsystem;
    protected IMU imu;
    protected Gamepad.LedEffect rgbLedEffect;
    @Override
    public void initialize() {
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        initHardware();
        setUpHardwareDevices();
        drive = new DriveSubsystem(fL, fR, bL, bR, imu);
        intakeSys = new IntakeSubsystem(intakeMotor, stackServo, () -> gamepadEx1.gamepad.right_trigger, () -> gamepadEx1.gamepad.left_trigger);
        launcherSubsystem = new LauncherSubsystem(launcherHeightServo, launcherServo);
        armSubsystem = new ArmSubsystem(pitchServo, armServo);
        liftSys = new LiftSubsystem(lil, lir, limitSwitchL, limitSwitchR,() -> gamepadEx1.gamepad.touchpad_finger_1_x, ()-> gamepadEx1.gamepad.touchpad_finger_1);
        boxSubsystem = new BoxSubsystem(clawL,clawR, gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        final CameraStreamProcessor processor = new CameraStreamProcessor();
        new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .build();
        FtcDashboard.getInstance().startCameraStream(processor, 0);
        telemetry.addData("Mode", "Done initializing");
        telemetry.update();
    }
    protected void initHardware() {
        stackServo = new SimpleServo(hardwareMap, "stackServo", 0 ,255);
        launcherHeightServo = new SimpleServo(hardwareMap, "launcherServo", 0,255);
        launcherServo = new SimpleServo(hardwareMap, "launch", 0,255);
        launcherHeightServo.setPosition(LauncherSubsystem.IDLE);
        pitchServo = new SimpleServo(hardwareMap, "pitch",0,270);
        clawL = new SimpleServo(hardwareMap, "clawL", 0,255);
        clawR = new SimpleServo(hardwareMap, "clawR", 0,255);
        armServo = new SimpleServo(hardwareMap, "axon", 0,255);
        limitSwitchL = hardwareMap.get(TouchSensor.class, "limitL");
        limitSwitchR = hardwareMap.get(TouchSensor.class, "limitR");

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
        tad("lil pos", lil.getCurrentPosition());
        tad("Target Position", liftSys.getTargetHeight());
        tad("MPPos", liftSys.getMPPos());
        tad("MPVel", liftSys.getMPVel());
        tad("intakeMotor power", round(intakeMotor.motorEx.getCurrent(CurrentUnit.AMPS)));
        tad("lir pos", lir.getCurrentPosition());
        tad("lil vel", lil.getVelocity());
        tad("lir vel", lir.getVelocity());
        tad("intakeServoPos", round(stackServo.getPosition()));
        tad("Limit Switch L", limitSwitchL.isPressed());
        tad("limit switch R", limitSwitchR.isPressed());
        tad("state", boxSubsystem.clawState);
        tad("touchpad x", gamepad1.touchpad_finger_1_x);
        tad("touchpad y", gamepad1.touchpad_finger_1_y);
        telemetry.update();
    }

    protected void setUpHardwareDevices() {
        fR.setInverted(true);
        bR.setInverted(true);
        intakeMotor.setInverted(true);
        lir.setInverted(true);
        lil.motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lir.motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lil.resetEncoder();
        lir.resetEncoder();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            run();
        }
        reset();
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
