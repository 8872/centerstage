package org.firstinspires.ftc.teamcode.util.ghost

import android.annotation.SuppressLint
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

    fun recordLeftStickY(lsticky: Double) {
        stickValues.setValue(StickValues.leftStickY, lsticky)
    }

    fun recordRightStickY(rsticky: Double) {
        stickValues.setValue(StickValues.rightStickY, rsticky)
    }

    fun recordLeftStickX(lstickx: Double) {
        stickValues.setValue(StickValues.leftStickX, lstickx)
    }

    fun recordRightStickX(rstickx: Double) {
        stickValues.setValue(StickValues.rightStickX, rstickx)
    }

    fun recordButtonX(value: Boolean) {
        buttonValues.setValue(ButtonValues.buttonX, value)
    }

    fun recordButtonY(value: Boolean) {
        buttonValues.setValue(ButtonValues.buttonY, value)
    }

    fun recordButtonA(value: Boolean) {
        buttonValues.setValue(ButtonValues.buttonA, value)
    }

    fun recordButtonB(value: Boolean) {
        buttonValues.setValue(ButtonValues.buttonB, value)
    }

    fun recordDpadUp(value: Boolean) {
        buttonValues.setValue(ButtonValues.dpadUp, value)
    }

    fun recordDpadDown(value: Boolean) {
        buttonValues.setValue(ButtonValues.dpadDown, value)
    }

    fun recordDpadLeft(value: Boolean) {
        buttonValues.setValue(ButtonValues.dpadLeft, value)
    }

    fun recordDpadRight(value: Boolean) {
        buttonValues.setValue(ButtonValues.dpadRight, value)
    }

    fun recordBumperLeft(value: Boolean) {
        buttonValues.setValue(ButtonValues.leftBumper, value)
    }

    fun recordBumperRight(value: Boolean) {
        buttonValues.setValue(ButtonValues.rightBumper, value)
    }

    fun recordLeftTrigger(value: Double) {
        triggerValues.setValue(TriggerValues.leftTrigger, value)
    }

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

    fun getString(): String {
        return instructions + updatesSinceValueChanged
    }

    @SuppressLint("SimpleDateFormat")
    fun save(content: String) {
        val dateFormat: DateFormat = SimpleDateFormat("MM-dd-yyyy_HH:mm:ss")
        val date: Date = Date()
        save(content, "code_generated_${dateFormat.format(date)}.mello")
    }

    @SuppressLint("SimpleDateFormat", "SdCardPath")
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