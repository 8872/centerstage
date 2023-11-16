package org.firstinspires.ftc.teamcode.util.sensors

import com.qualcomm.hardware.broadcom.BroadcomColorSensorImpl
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType
import com.qualcomm.robotcore.util.RobotLog
import com.qualcomm.robotcore.util.TypeConversion
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

@I2cDeviceType
@DeviceProperties(
        name = "MB1242",
        description = "ultrasonic distance sensor",
        xmlTag = "MB1242Dev"
)
class MB1242Ex : I2cDeviceSynchDevice<I2cDeviceSynch>, DistanceSensor {
    private var lastRun: Long = 0
    private var runDelayMs: Long = 100
    private var lastReading: Short = 0
    private var a = 0.8
    private var enabled = 0x00;

    constructor(hardwareMap: HardwareMap, name: String) : this(hardwareMap.i2cDeviceSynch.get(name))

    constructor(i2cDeviceSynch: I2cDeviceSynch) : super(i2cDeviceSynch, true) {
        deviceClient.setI2cAddress(I2cAddr.create7bit(0x70))
        registerArmingStateCallback(false)
        deviceClient.engage()
    }

    override fun getDistance(unit: DistanceUnit): Double {
        if (System.currentTimeMillis() > (lastRun + runDelayMs)) {
            lastRun = System.currentTimeMillis()
            deviceClient.write(TypeConversion.intToByteArray(0x51))
        }

        if (System.currentTimeMillis() > (lastRun + 20)) {
            lastReading = TypeConversion.byteArrayToShort(deviceClient.read(2))
        }

        return lowPassFilter(unit.fromCm(lastReading.toDouble()) + 2)
    }

    override fun getManufacturer(): HardwareDevice.Manufacturer {
        return HardwareDevice.Manufacturer.Other
    }

    fun setRunDelayMs(runDelayMs: Long) {
        this.runDelayMs = runDelayMs
    }
    override fun getDeviceName(): String {
        return "MB1242 Distance Sensor"
    }

    override fun getConnectionInfo(): String {
        return "Connected..."
    }

    override fun getVersion(): Int {
        return 1
    }

    override fun doInitialize(): Boolean {
        RobotLog.vv(BroadcomColorSensorImpl.TAG, "enable() enabled=0x%02x...", enabled)
        return true
    }

    override fun resetDeviceConfigurationForOpMode() {
        super.resetDeviceConfigurationForOpMode()
    }

    override fun close() {
        RobotLog.vv(BroadcomColorSensorImpl.TAG, "close()")
    }

    private fun lowPassFilter(measurement: Double): Double {
        return (a * lastReading) + (1 - a) * measurement
    }

    fun getLog(): String {
        return deviceClient.connectionInfo
    }
}