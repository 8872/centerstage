package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@I2cDeviceType
@DeviceProperties(
        name = "MB1242",
        description = "ultrasonic distance sensor",
        xmlTag = "MB1242"
)
public class MB1242 extends I2cDeviceSynchDevice<I2cDeviceSynch> implements DistanceSensor {
    private long lastRun = 0;
    private long runDelayMs = 100;
    private short lastReading = 0;
    private double a = 0.8;

    public MB1242(HardwareMap hardwareMap, String name) {
        this(hardwareMap.i2cDeviceSynch.get(name));
    }

    public MB1242(I2cDeviceSynch i2cDeviceSynch) {
        super(i2cDeviceSynch, true);

        deviceClient.setI2cAddress(I2cAddr.create7bit(0x70));
        registerArmingStateCallback(false);
        deviceClient.engage();
    }

    @Override
    public double getDistance(DistanceUnit unit) {
        if (System.currentTimeMillis() > (lastRun + runDelayMs)) {
            lastRun = System.currentTimeMillis();
            deviceClient.write(TypeConversion.intToByteArray(0x51));
        }

        if (System.currentTimeMillis() > (lastRun + 20)) {
            lastReading = TypeConversion.byteArrayToShort(deviceClient.read(2));
        }

        return lowPassFilter(unit.fromCm(lastReading) + 2);
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "MB1242 Distance Sensor";
    }

    @Override
    public String getConnectionInfo() {
        return "idfk bro";
    }

    @Override
    public int getVersion() {
        return 1;
    }

    @Override
    protected boolean doInitialize() {
        return true;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }

    private double lowPassFilter(double measurement) {
        return (a * lastReading) + (1 - a) * measurement;
    }
}
