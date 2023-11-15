package org.firstinspires.ftc.teamcode.util.ghost

class ButtonValues() : ControllerValues<Boolean>(
        false,
        arrayListOf(false, false, false, false, false, false, false, false, false, false),
        arrayListOf(buttonA, buttonB, buttonX, buttonY, dpadUp, dpadDown, dpadLeft, dpadRight, leftBumper, rightBumper)
) {
    companion object {
        const val buttonA = "a"
        const val buttonB = "b"
        const val buttonX = "x"
        const val buttonY = "y"
        const val dpadUp = "du"
        const val dpadDown = "dd"
        const val dpadLeft = "dl"
        const val dpadRight = "dr"
        const val leftBumper = "lb"
        const val rightBumper = "rb"
    }

}