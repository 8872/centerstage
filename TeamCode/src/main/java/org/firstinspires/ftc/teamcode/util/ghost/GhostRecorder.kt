package org.firstinspires.ftc.teamcode.util.ghost


import com.qualcomm.robotcore.hardware.Gamepad
import java.io.File
import java.io.FileWriter
import java.io.IOException
import java.text.DateFormat
import java.text.SimpleDateFormat
import java.util.*

class GhostRecorder {
    private var instructions = ""
    private var updatesSinceValueChanged = 0
    private val stickValues = StickValues()
    private val buttonValues = ButtonValues()
    private val triggerValues = TriggerValues()  // Added TriggerValues

    /**
     * Records the values of the gamepad
     */
    fun recordGamepad(gamepad: Gamepad) {
        recordLeftStickY(gamepad.left_stick_y.toDouble())
        recordRightStickY(gamepad.right_stick_y.toDouble())
        recordLeftStickX(gamepad.left_stick_x.toDouble())
        recordRightStickX(gamepad.right_stick_x.toDouble())
        recordButtonX(gamepad.x)
        recordButtonY(gamepad.y)
        recordButtonA(gamepad.a)
        recordButtonB(gamepad.b)
        recordDpadUp(gamepad.dpad_up)
        recordDpadDown(gamepad.dpad_down)
        recordDpadLeft(gamepad.dpad_left)
        recordDpadRight(gamepad.dpad_right)
        recordBumperLeft(gamepad.left_bumper)
        recordBumperRight(gamepad.right_bumper)
        recordLeftTrigger(gamepad.left_trigger.toDouble())
        recordRightTrigger(gamepad.right_trigger.toDouble())
    }

    /**
     * Records the values of the left stick y axis
     * @param lsticky
     */
    fun recordLeftStickY(lsticky: Double) {
        stickValues.setValue(StickValues.leftStickY, lsticky)
    }
    /**
     * Records the values of the right stick y axis
     * @param rsticky
     */
    fun recordRightStickY(rsticky: Double) {
        stickValues.setValue(StickValues.rightStickY, rsticky)
    }
    /**
     * Records the values of the left stick x axis
     * @param lstickx
     */
    fun recordLeftStickX(lstickx: Double) {
        stickValues.setValue(StickValues.leftStickX, lstickx)
    }
    /**
     * Records the values of the right stick x axis
     * @param rstickx
     */
    fun recordRightStickX(rstickx: Double) {
        stickValues.setValue(StickValues.rightStickX, rstickx)
    }
    /**
     * Records the values of the button x
     *  @param value
     */
    fun recordButtonX(value: Boolean) {
        buttonValues.setValue(ButtonValues.buttonX, value)
    }
    /**
     * Records the values of the button y
     * @param value
     */
    fun recordButtonY(value: Boolean) {
        buttonValues.setValue(ButtonValues.buttonY, value)
    }
    /**
     * Records the values of the button a
     * @param value
     */
    fun recordButtonA(value: Boolean) {
        buttonValues.setValue(ButtonValues.buttonA, value)
    }
    /**
     * Records the values of the button b
     * @param value
     */
    fun recordButtonB(value: Boolean) {
        buttonValues.setValue(ButtonValues.buttonB, value)
    }
    /**
     * Records the values of the dpad up
     *  @param value
     */
    fun recordDpadUp(value: Boolean) {
        buttonValues.setValue(ButtonValues.dpadUp, value)
    }
    /**
     * Records the values of the dpad down
     * @param value
     */
    fun recordDpadDown(value: Boolean) {
        buttonValues.setValue(ButtonValues.dpadDown, value)
    }
    /**
     * Records the values of the dpad left
     * @param value
     */
    fun recordDpadLeft(value: Boolean) {
        buttonValues.setValue(ButtonValues.dpadLeft, value)
    }
    /**
     * Records the values of the dpad right
     *  @param value
     */
    fun recordDpadRight(value: Boolean) {
        buttonValues.setValue(ButtonValues.dpadRight, value)
    }
    /**
     * Records the values of the left bumper
     * @param value
     */
    fun recordBumperLeft(value: Boolean) {
        buttonValues.setValue(ButtonValues.leftBumper, value)
    }
    /**
     * Records the values of the right bumper
     * @param value
     */
    fun recordBumperRight(value: Boolean) {
        buttonValues.setValue(ButtonValues.rightBumper, value)
    }
    /**
     * Records the values of the left trigger
     *  @param value
     */
    fun recordLeftTrigger(value: Double) {
        triggerValues.setValue(TriggerValues.leftTrigger, value)
    }
    /**
     * Records the values of the right trigger
     * @param value
     */
    fun recordRightTrigger(value: Double) {
        triggerValues.setValue(TriggerValues.rightTrigger, value)
    }

    fun getStringOfChangedVals(vals: StickValues): String {
        var line = ""
        val syms = vals.getSymbolsOfChanged()
        for (sym in syms) {
            line += "$sym:${vals.getValue(sym)} "
        }
        return line
    }

    fun getStringOfChangedVals(vals: ButtonValues): String {
        var line = ""
        val syms = vals.getSymbolsOfChanged()
        for (sym in syms) {
            line += "$sym:${vals.getValue(sym)} "
        }
        return line
    }

    fun getStringOfChangedVals(vals: TriggerValues): String {
        var line = ""
        val syms = vals.getSymbolsOfChanged()
        for (sym in syms) {
            line += "$sym:${vals.getValue(sym)} "
        }
        return line
    }
    /**
     * Updates the instructions
     */
    fun update() {
        val line = getStringOfChangedVals(stickValues) + getStringOfChangedVals(buttonValues) +
                getStringOfChangedVals(triggerValues) // Added trigger values
        if (line.isNotEmpty()) {
            if (updatesSinceValueChanged > 0) {
                instructions += "$updatesSinceValueChanged "
            }
            instructions += line
            updatesSinceValueChanged = 0
        }

        updatesSinceValueChanged += 1
    }
    /**
     * updates the instructions and records the gamepad at the same time
     * functionally equivalent to calling recordGamepad and update separately
     * @param gamepad the gamepad to record
     */
    fun update(gamepad: Gamepad) {
        recordGamepad(gamepad)
        update()
    }

    fun getString(): String {
        return instructions + updatesSinceValueChanged
    }
    /**
     * Saves the instructions to a file
     * @param content the content to save
     */
    fun save(content: String) {
        val dateFormat: DateFormat = SimpleDateFormat("MM-dd-yyyy_HH:mm:ss")
        val date: Date = Date()
        save(content, "code_generated_${dateFormat.format(date)}.mello")
    }
    /**
     * Saves the instructions to a file with a given name
     * @param content the content to save
     * @param name the name of the file
     */
    fun save(content: String, name: String) {
        val externalStorageDir = File("/sdcard/FIRST/")
        val dateFormat: DateFormat = SimpleDateFormat("MM-dd-yyyy_HH:mm:ss")
        val date: Date = Date()
        val appDir = File(externalStorageDir, "Ghost")
        if (!appDir.exists()) {
            appDir.mkdirs()
        }
        val file = File(appDir, "$name.mello")

        try {
            val writer = FileWriter(file)
            writer.append(content)
            writer.flush()
            writer.close()
        } catch (e: IOException) {
            e.printStackTrace()
        }
    }
}