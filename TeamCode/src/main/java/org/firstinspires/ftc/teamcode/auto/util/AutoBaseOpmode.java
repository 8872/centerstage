package org.firstinspires.ftc.teamcode.auto.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.auto.CV.ZoneDetectionProcessorRight;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.*;
import org.firstinspires.ftc.teamcode.util.MB1242;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

public class AutoBaseOpmode extends OpMode {
    protected TouchSensor limitSwitch;
    protected MB1242 flSensor, frSensor, blSensor;
    protected SimpleServo armServo, pitchServo, innerServo, outerServo, stack, stack2, plane;
    protected MotorEx leftFront, leftRear, rightRear, rightFront, liftLeft, liftRight, hang, intake;

    protected DistanceSensor beam, beam2;

    protected ArmSubsystem armSubsystem;
    protected BoxSubsystem boxSubsystem;
    protected DriveSubsystem driveSubsystem;
    protected IntakeSubsystem intakeSubsystem;
    protected LocalizerSubsystem localizerSubsystem;
    protected PlaneSubsystem planeSubsystem;
    protected LiftSubsystem liftSubsystem;
    List<LynxModule> hubs;

    protected SampleMecanumDrive drive;

    protected VisionPortal portal;
    protected ZoneDetectionProcessorRight processor;
    protected boolean red;
    protected boolean right;

//    protected AutoBaseOpmode(boolean red, boolean right) {
//        this.red = red;
//        this.right = right;
//    }

    @Override
    public void init() {
        initHardware();
        setupHardware();
        initSubystems();
        setupMisc();
        telemetry.addData("Mode", "Done initializing");
        telemetry.update();
    }

    public void initHardware() {
        limitSwitch = hardwareMap.get(TouchSensor.class, "limit");
        flSensor = hardwareMap.get(MB1242.class, "flSensor");
        frSensor = hardwareMap.get(MB1242.class, "frSensor");
        blSensor = hardwareMap.get(MB1242.class, "blSensor");

        beam = hardwareMap.get(DistanceSensor.class, "beam");
        beam2 = hardwareMap.get(DistanceSensor.class, "beam2");

        plane = new SimpleServo(hardwareMap, "airplane", 0, 255);
        armServo = new SimpleServo(hardwareMap, "armServo", 0, 355);
        pitchServo = new SimpleServo(hardwareMap, "pitchServo", 0, 355);
        innerServo = new SimpleServo(hardwareMap, "innerServo", 0, 255);
        outerServo = new SimpleServo(hardwareMap, "outerServo", 0, 255);
        stack = new SimpleServo(hardwareMap, "stack", 0, 255);
        stack2 = new SimpleServo(hardwareMap, "stack2", 0, 255);
        stack2.setInverted(true);

        leftFront = new MotorEx(hardwareMap, "rightRear", Motor.GoBILDA.RPM_435);
        leftRear = new MotorEx(hardwareMap, "leftFront", Motor.GoBILDA.RPM_435);
        rightRear = new MotorEx(hardwareMap, "rightFront", Motor.GoBILDA.RPM_435);
        rightFront = new MotorEx(hardwareMap, "leftRear", Motor.GoBILDA.RPM_435);
        intake = new MotorEx(hardwareMap, "intake", Motor.GoBILDA.RPM_1150);
        liftLeft = new MotorEx(hardwareMap, "lil", Motor.GoBILDA.RPM_1150);
        liftRight = new MotorEx(hardwareMap, "lir", Motor.GoBILDA.RPM_1150);
        hang = new MotorEx(hardwareMap, "hang", Motor.GoBILDA.RPM_30);
    }

    public void setupHardware() {
        liftLeft.setInverted(true);
        leftRear.setInverted(true);
        leftFront.setInverted(true);
    }

    public void initSubystems() {
        liftSubsystem = new LiftSubsystem(liftLeft, liftRight, limitSwitch, hardwareMap.voltageSensor, () -> 0);
        localizerSubsystem = new LocalizerSubsystem(flSensor, frSensor, blSensor);
        armSubsystem = new ArmSubsystem(armServo, pitchServo);
        armSubsystem.intake();

        boxSubsystem = new BoxSubsystem(innerServo, outerServo);
        driveSubsystem = new DriveSubsystem(leftFront, rightFront, leftRear, rightRear);
        intakeSubsystem = new IntakeSubsystem(stack, stack2, intake, hardwareMap.voltageSensor);
        planeSubsystem = new PlaneSubsystem(plane);
        intakeSubsystem.setStack1(IntakeSubsystem.servoHighPosition);
        intakeSubsystem.setStack2(IntakeSubsystem.servoHighPosition);
        intakeSubsystem.runIntake(0);
    }

    public void setupMisc() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        final CameraStreamProcessor processor = new CameraStreamProcessor();
//        new VisionPortal.Builder()
//                .addProcessor(processor)
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .build();

//        processor = new ZoneDetectionProcessorRight(red, right);
//        portal = new VisionPortal.Builder()
//                .addProcessor(processor)
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
//                .setCameraResolution(new Size(1280, 720))
//                .setAutoStopLiveView(true)
//                .enableLiveView(true)
//                .build();

//        FtcDashboard.getInstance().startCameraStream(processor, 0);
        drive = new SampleMecanumDrive(hardwareMap);
    }

    //    protected GamepadButton gb1(GamepadKeys.Button button) {
//        return gamepadEx1.getGamepadButton(button);
//    }
//    protected GamepadButton gb2(GamepadKeys.Button button) {
//        return gamepadEx2.getGamepadButton(button);
//    }
    protected void tad(String caption, Object value) {
        telemetry.addData(caption, value);
    }

    public void schedule(Command... commands) {
        CommandScheduler.getInstance().schedule(commands);
    }


//    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
//        private final AtomicReference<Bitmap> lastFrame =
//                new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
//
//        @Override
//        public void init(int width, int height, CameraCalibration calibration) {
//            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
//        }
//
//        @Override
//        public Object processFrame(Mat frame, long captureTimeNanos) {
//            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
//            Utils.matToBitmap(frame, b);
//            lastFrame.set(b);
//            return null;
//        }
//
//        @Override
//        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
//                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
//                                Object userContext) {
//            // do nothing
//        }
//
//        @Override
//        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
//            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
//        }
//    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
    }
}