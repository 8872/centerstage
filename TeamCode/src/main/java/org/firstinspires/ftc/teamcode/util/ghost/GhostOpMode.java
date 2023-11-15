package org.firstinspires.ftc.teamcode.util.ghost;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp(name="DriveRecorder", group="Iterative Opmode")
public class GhostOpMode extends OpMode {
    /**
     * First Controller Recorder
     */
    GhostRecorder ghostRecorderC1 =new GhostRecorder();
    /**
     * Second Controller Recorder
     */
    GhostRecorder ghostRecorderC2 =new GhostRecorder();
    /**
     * First Controller Replayer
     */
    GhostController ghostController = GhostController.Companion.loadFromFile("firstController.mello");
    /**
     * Second Controller Replayer
     */
    GhostController ghostController2 = GhostController.Companion.loadFromFile("secondController.mello");
    /**
     * Multiple Controller Replays
     */
    GhostControllers ghostControllers = new GhostControllers(ghostController, ghostController2);

    public void init() {
    }

    public void loop() {
        ghostController.update();
        ghostController2.update();
        gamepad1 = ghostController.fakeGamepad();
        gamepad2 = ghostController2.fakeGamepad();

        ghostControllers.update();
        gamepad1 = ghostControllers.get(0).fakeGamepad();
        gamepad2 = ghostControllers.get(1).fakeGamepad();


        ghostRecorderC1.update(gamepad1); //have to to this to actually record all of the values
        ghostRecorderC2.update(gamepad2);

    }

    public void stop() {
        telemetry.addData("Stopped", ghostRecorderC1.getString());
        telemetry.update();
        ghostRecorderC1.save(ghostRecorderC1.getString(), "firstController.mello");
        ghostRecorderC2.save(ghostRecorderC2.getString(), "secondController.mello");
    }

}
