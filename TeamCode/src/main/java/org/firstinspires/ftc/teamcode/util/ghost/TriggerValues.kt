package org.firstinspires.ftc.teamcode.util.ghost


class TriggerValues : ControllerValues<Double>(
        0.0,
        arrayListOf(0.0, 0.0),
        arrayListOf(leftTrigger, rightTrigger)
) {
    companion object {
        const val leftTrigger = "lt"
        const val rightTrigger = "rt"
    }

}
