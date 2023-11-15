package org.firstinspires.ftc.teamcode.util.ghost

class StickValues() : ControllerValues<Double>(
        0.0,
        arrayListOf(0.0, 0.0, 0.0, 0.0),
        arrayListOf(leftStickY, rightStickY, leftStickX, rightStickX)
) {
    companion object {
        const val leftStickY = "ly"
        const val rightStickY = "ry"
        const val leftStickX = "lx"
        const val rightStickX = "rx"
    }

}