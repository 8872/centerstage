package org.firstinspires.ftc.teamcode.util.ghost

import android.annotation.SuppressLint
import java.io.BufferedReader
import java.io.File
import java.io.FileReader
import java.io.IOException

class GhostController(private val instructions: String) {
    val stickValues = StickValues()
    val buttonValues = ButtonValues()
    val triggerValues = TriggerValues()  // Added TriggerValues
    private var pos = 0
    private var waitFor = 0

    companion object {
        fun loadFromFile(fileName: String): GhostController {
            val content = StringBuilder()
            try {
                @SuppressLint("SdCardPath") val directoryPath = "/sdcard/FIRST/Ghost"
                val file = File(directoryPath, fileName)
                BufferedReader(FileReader(file)).use { br ->
                    var line: String?
                    while (br.readLine().also { line = it } != null) {
                        content.append(line).append("\n")
                    }
                }
            } catch (e: IOException) {
                e.printStackTrace()
            }
            return GhostController(content.toString())
        }
    }

    private fun getNextValue(): String {
        var value = ""
        while (pos < instructions.length) {
            when {
                instructions[pos] == ' ' -> {
                    pos += 1
                    return value
                }
                pos == instructions.length - 1 -> {
                    value += instructions[pos]
                    pos += 1
                    return value
                }
                else -> value += instructions[pos]
            }
            pos += 1
        }
        return ""
    }

    fun areInstructionsLeft(): Boolean {
        return pos < instructions.length
    }

    fun update() {
        if (areInstructionsLeft()) {
            if (waitFor == 0) {
                val valString = getNextValue()
                var beforeColon = ""
                var afterColon = ""
                var foundColon = false
                for (i in valString.indices) {
                    if (!foundColon) {
                        if (valString[i] == ':') {
                            foundColon = true
                        } else {
                            beforeColon += valString[i]
                        }
                    } else {
                        afterColon += valString[i]
                    }
                }

                if (foundColon) {
                    when {
                        afterColon == "true" -> buttonValues.setValue(beforeColon, true)
                        afterColon == "false" -> buttonValues.setValue(beforeColon, false)
                        beforeColon == TriggerValues.leftTrigger || beforeColon == TriggerValues.rightTrigger ->
                            triggerValues.setValue(beforeColon, afterColon.toDouble())
                        else -> stickValues.setValue(beforeColon, afterColon.toDouble())
                    }

                    update()
                } else {
                    waitFor = beforeColon.toInt() - 1
                }
            } else {
                waitFor -= 1
            }
        } else {
            if (waitFor > 0) {
                waitFor -= 1
            } else {
                buttonValues.reset()
                stickValues.reset()
                triggerValues.reset()  // Reset trigger values
            }
        }
    }

    fun leftStickY(): Double {
        return stickValues.getValue(StickValues.leftStickY)
    }

    fun leftStickX(): Double {
        return stickValues.getValue(StickValues.leftStickX)
    }

    fun rightStickY(): Double {
        return stickValues.getValue(StickValues.rightStickY)
    }

    fun rightStickX(): Double {
        return stickValues.getValue(StickValues.rightStickX)
    }

    fun buttonA(): Boolean {
        return buttonValues.getValue(ButtonValues.buttonA)
    }

    fun buttonB(): Boolean {
        return buttonValues.getValue(ButtonValues.buttonB)
    }

    fun buttonX(): Boolean {
        return buttonValues.getValue(ButtonValues.buttonX)
    }

    fun buttonY(): Boolean {
        return buttonValues.getValue(ButtonValues.buttonY)
    }

    fun dpadUp(): Boolean {
        return buttonValues.getValue(ButtonValues.dpadUp)
    }

    fun dpadDown(): Boolean {
        return buttonValues.getValue(ButtonValues.dpadDown)
    }

    fun dpadLeft(): Boolean {
        return buttonValues.getValue(ButtonValues.dpadLeft)
    }

    fun dpadRight(): Boolean {
        return buttonValues.getValue(ButtonValues.dpadRight)
    }

    fun bumperLeft(): Boolean {
        return buttonValues.getValue(ButtonValues.leftBumper)
    }

    fun bumperRight(): Boolean {
        return buttonValues.getValue(ButtonValues.rightBumper)
    }

    fun leftTrigger(): Double {
        return triggerValues.getValue(TriggerValues.leftTrigger)
    }

    fun rightTrigger(): Double {
        return triggerValues.getValue(TriggerValues.rightTrigger)
    }
}
