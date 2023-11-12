package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.ghost.CodeSharer;
import org.firstinspires.ftc.teamcode.util.ghost.GhostRecorder;
@TeleOp(name="DriveRecorder", group="Iterative Opmode")
class GhostOpMode extends OpMode {

    GhostRecorder ghostRecorder=new GhostRecorder();
    CodeSharer codeSharer=new CodeSharer(hardwareMap.appContext);

    public void init() {
    }

    public void loop() {

        //only record values you are using to control robot
        ghostRecorder.recordLeftStickY(gamepad1.left_stick_y);
        ghostRecorder.recordRightStickY(gamepad1.right_stick_y);
        ghostRecorder.recordLeftStickX(gamepad1.left_stick_x);
        ghostRecorder.recordRightStickX(gamepad1.right_stick_x);

        ghostRecorder.recordButtonA(gamepad1.x);
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
        codeSharer.save(ghostRecorder.getString());
    }
}
