package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.auto.opmodes.TestBaseOpMode;
import org.firstinspires.ftc.teamcode.subsystem.*;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;


import android.graphics.Bitmap;
import android.graphics.Canvas;

import java.util.concurrent.atomic.AtomicReference;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;

import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.math.BigDecimal;
import java.math.RoundingMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.auto.opmodes.TestBaseOpMode;
import org.firstinspires.ftc.teamcode.subsystem.*;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;


import android.graphics.Bitmap;
import android.graphics.Canvas;

import java.util.concurrent.atomic.AtomicReference;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;

import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.math.BigDecimal;
import java.math.RoundingMode;

@Config
public class DriveBaseOpMode extends TestBaseOpMode {

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

    protected MotorEx frontLeft, frontRight, backLeft, backRight, intakeMotor, liftLeft, liftRight;
    protected SimpleServo stackServo, launcherHeightServo, launcherReleaseServo, pitchServo, clawLeft, clawRight, armServo;
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

        drive = new DriveSubsystem(frontLeft, frontRight, backLeft, backRight, imu);
        intakeSys = new IntakeSubsystem(intakeMotor, stackServo, () -> gamepadEx2.gamepad.right_trigger, () -> gamepadEx2.gamepad.left_trigger);
        launcherSubsystem = new LauncherSubsystem(launcherHeightServo, launcherReleaseServo);
        armSubsystem = new ArmSubsystem(pitchServo, armServo);
        liftSys = new LiftSubsystem(liftLeft, liftRight, limitSwitchL, limitSwitchR);
        boxSubsystem = new BoxSubsystem(clawLeft, clawRight);

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
        stackServo = new SimpleServo(hardwareMap, "stackServo", 0, 255);
        intakeMotor = new MotorEx(hardwareMap, "intakeMotor");

        launcherHeightServo = new SimpleServo(hardwareMap, "launcherServo", 0, 255);
        launcherReleaseServo = new SimpleServo(hardwareMap, "launch", 0, 255);

        clawLeft = new SimpleServo(hardwareMap, "clawL", 0, 255);
        clawRight = new SimpleServo(hardwareMap, "clawR", 0, 255);

        armServo = new SimpleServo(hardwareMap, "axon", 0, 255);
        pitchServo = new SimpleServo(hardwareMap, "pitch", 0, 270);

        limitSwitchL = hardwareMap.get(TouchSensor.class, "limitL");
        limitSwitchR = hardwareMap.get(TouchSensor.class, "limitR");

        frontLeft = new MotorEx(hardwareMap, "fl");
        frontRight = new MotorEx(hardwareMap, "fr");
        backLeft = new MotorEx(hardwareMap, "bl");
        backRight = new MotorEx(hardwareMap, "br");

        liftLeft = new MotorEx(hardwareMap, "lil");
        liftRight = new MotorEx(hardwareMap, "lir");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.DOWN))
        );
    }

    @Override
    public void run() {
        super.run();

        tad("frontLeft Power", frontLeft.get());
        tad("frontRight Power", frontRight.get());
        tad("backLeft Power", backLeft.get());
        tad("backRight Power", backRight.get());

        tad("Lift Target Position", liftSys.getTargetHeight());
        tad("liftLeft Position", liftLeft.getCurrentPosition());
        tad("liftRight Position", liftRight.getCurrentPosition());

        tad("launcherHeight Position", launcherHeightServo.getPosition());
        tad("launcherRelease Position", launcherReleaseServo.getPosition());

        tad("Limit Switch L", limitSwitchL.isPressed());
        tad("Limit Switch R", limitSwitchR.isPressed());

        tad("firstClosed", boxSubsystem.secondClosed());
        tad("gb2 bumperRight", gamepadEx2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).get());
        tad("intakeMotor power", round(intakeMotor.motorEx.getCurrent(CurrentUnit.AMPS)));
        tad("intakeServoPos", round(stackServo.getPosition()));

        tad("state", boxSubsystem.clawState);
        tad("touchpad x", gamepad1.touchpad_finger_1_x);
        tad("touchpad y", gamepad1.touchpad_finger_1_y);
        telemetry.update();
    }

    protected void setUpHardwareDevices() {
        frontRight.setInverted(true);
        backRight.setInverted(true);
        intakeMotor.setInverted(true);
        liftRight.setInverted(true);
        liftLeft.motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftLeft.resetEncoder();
        liftRight.resetEncoder();
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
    protected GamepadButton gb1(GamepadKeys.Button button) {
        return gamepadEx1.getGamepadButton(button);
    }

    protected GamepadButton gb2(GamepadKeys.Button button) {
        return gamepadEx2.getGamepadButton(button);
    }

    // telemetry add data = tad
    protected void tad(String caption, Object value) {
        telemetry.addData(caption, value);
    }
}