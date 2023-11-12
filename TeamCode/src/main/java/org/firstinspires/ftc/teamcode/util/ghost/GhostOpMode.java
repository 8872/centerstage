package org.firstinspires.ftc.teamcode.util.ghost;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.ghost.GhostRecorder;
@TeleOp(name="DriveRecorder", group="Iterative Opmode")
public class GhostOpMode extends OpMode {

    GhostRecorder ghostRecorder=new GhostRecorder();

    public void init() {
    }

    public void loop() {

        ghostRecorder.recordButtonA(gamepad1.a);
        ghostRecorder.recordButtonX(gamepad1.b);
        ghostRecorder.recordButtonY(gamepad1.y);

        ghostRecorder.recordLeftTrigger(gamepad1.left_trigger);
        ghostRecorder.recordRightTrigger(gamepad1.right_trigger);

        //and B,X or Y

        ghostRecorder.recordDpadDown(gamepad1.dpad_down);
        //and Up,Left or Right

        //code to drive robot
        //...

        ghostRecorder.update(); //have to to this to actually record all of the values
        telemetry.addData("Recording", ghostRecorder.getString());
        telemetry.update();
    }

    public void stop() {
        telemetry.addData("Stopped", ghostRecorder.getString());
        telemetry.update();
        ghostRecorder.save(ghostRecorder.getString());
    }

}
