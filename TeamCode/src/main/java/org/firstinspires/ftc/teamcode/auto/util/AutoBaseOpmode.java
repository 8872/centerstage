package org.firstinspires.ftc.teamcode.auto.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
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
    protected GamepadEx gamepadEx1, gamepadEx2;
    protected SimpleServo armServo, pitchServo, innerServo, outerServo, stack, stack2, plane;
    protected MotorEx leftFront, leftRear, rightRear, rightFront, liftLeft, liftRight, hang, intake;
    protected RevBlinkinLedDriver blinkin;
    protected DistanceSensor beam, beam2;

    protected HangSubsystem hangSubsystem;
    protected ArmSubsystem armSubsystem;
    protected BoxSubsystem boxSubsystem;
    protected DriveSubsystem driveSubsystem;
    protected IntakeSubsystem intakeSubsystem;
    protected LocalizerSubsystem localizerSubsystem;
    protected PlaneSubsystem planeSubsystem;
    protected LiftSubsystem liftSubsystem;
    protected BlinkinSubsystem blinkinSubsystem;
    protected SampleMecanumDrive drive;
    List<LynxModule> hubs;

    @Override
    public void init() {
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        initHardware();
        setupHardware();
        initSubystems();
        setupMisc();
        telemetry.addData("Mode", "Done initializing");
        telemetry.update();
    }

    public void initHardware() {
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
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
        stack.setInverted(true);

        leftFront = new MotorEx(hardwareMap, "leftFront", Motor.GoBILDA.RPM_435);
        leftRear = new MotorEx(hardwareMap, "leftRear", Motor.GoBILDA.RPM_435);
        rightRear = new MotorEx(hardwareMap, "rightRear", Motor.GoBILDA.RPM_435);
        rightFront = new MotorEx(hardwareMap, "rightFront", Motor.GoBILDA.RPM_435);
        intake = new MotorEx(hardwareMap, "intake", Motor.GoBILDA.RPM_1150);
        liftLeft = new MotorEx(hardwareMap, "lil", Motor.GoBILDA.RPM_435);
        liftRight = new MotorEx(hardwareMap, "lir", Motor.GoBILDA.RPM_435);
        hang = new MotorEx(hardwareMap, "hang", Motor.GoBILDA.RPM_30);
    }

    public void setupHardware() {
        liftRight.encoder.setDirection(Motor.Direction.REVERSE);
    }

    public void initSubystems() {
        hangSubsystem = new HangSubsystem(hang, gamepadEx2::getLeftY);
        liftSubsystem = new LiftSubsystem(liftLeft, liftRight, limitSwitch, hardwareMap.voltageSensor, () -> -gamepadEx2.getRightY());
        localizerSubsystem = new LocalizerSubsystem(flSensor, frSensor, blSensor);
        armSubsystem = new ArmSubsystem(armServo, pitchServo);
        armSubsystem.intake();
        blinkinSubsystem = new BlinkinSubsystem(blinkin);
        boxSubsystem = new BoxSubsystem(innerServo, outerServo, blinkinSubsystem);
        driveSubsystem = new DriveSubsystem(leftFront, rightFront, leftRear, rightRear);
        intakeSubsystem = new IntakeSubsystem(stack, stack2, intake, hardwareMap.voltageSensor, blinkinSubsystem);
        planeSubsystem = new PlaneSubsystem(plane);
    }

    public void setupMisc() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);
    }

    protected void tad(String caption, Object value) {
        telemetry.addData(caption, value);
    }

    public void schedule(Command... commands) {
        CommandScheduler.getInstance().schedule(commands);
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        tad("bl reading:", localizerSubsystem.getBl());
        telemetry.update();

    }
}