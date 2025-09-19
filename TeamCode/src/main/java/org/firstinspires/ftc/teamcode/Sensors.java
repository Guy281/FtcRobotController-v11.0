// This template is a starting point on setting up your sensors
// If you are not using anything listed below, comment them out (like this line)
// Deleting them is fine too but commenting is recommended in case they are added later

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//If using navX sensor (needs external libraries)
//import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
//import com.kauailabs.navx.ftc.AHRS;

public class Sensors {
    // TODO: If using a limit switch, magnetic limit switch, or touch sensor, add them here
    //public DigitalChannel limitSwitch;
    //public DigitalChannel touchSensor;
    //public DigitalChannel magneticLimitSwitch;
    // TODO: If using an analog sensor (for example, a potentiometer), add them here
    //public AnalogInput potSensor;
    // TODO: If using a color sensor, add them here
    //public ColorSensor colorSensor;
    // TODO: If using a 2M Distance Sensor, add them here
    //public DistanceSensor distanceSensor;
    // TODO: If using the built-in Control Hub IMU gyroscope, include this
    public IMU imu;
    // TODO: If using the navX micro sensor, include this
    //public AHRS navx_device;

    public void init(HardwareMap hwMap) {
        //TODO: Put sensor hardware here, don't forget to rename deviceNames to match yours
        //TODO: If using limit switches or other digital sensors
        //limitSwitch = hwMap.get(DigitalChannel.class, "limitSwitch");
        //limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        //touchSensor = hwMap.get(DigitalChannel.class, "touchSensor");
        //touchSensor.setMode(DigitalChannel.Mode.INPUT);
        //magneticLimitSwitch = hwMap.get(DigitalChannel.class, "magneticLimitSwitch");
        //magneticLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        //TODO: If using analog sensors
        //potSensor = hwMap.get(AnalogInput.class, "potSensor");
        //TODO: If using color sensors
        //colorSensor = hwMap.get(ColorSensor.class, "colorSensor");
        //TODO: If using distance sensors
        //distanceSensor = hwMap.get(DistanceSensor.class, "distanceSensor");
        //TODO: If using the IMU
        imu = hwMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot RevOrientation =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.
                        LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
            imu.initialize(new IMU.Parameters(RevOrientation));
        //TODO: If using the navX-micro sensor
        // navx_device = AHRS.getInstance(hwMap.get(NavxMicroNavigationSensor.class, "navx"),
        // AHRS.DeviceDataType.kProcessedData);
    }

    //TODO: Sample class for checking limit switch state (true/false)
    //public boolean getLimitSwitchState() { return !limitSwitch.getState(); }

    //TODO: Sample class for checking magnetic limit switch state (true/false)
    //public boolean getMagneticLimitSwitchState() { return !magneticLimitSwitch.getState(); }

    //TODO: Sample class for checking touch sensor (true/false)
    //public boolean getTouchSensorState() { return !touchSensor.getState(); }

    //TODO: Sample class for checking potentiometer angle
    //public double getPotAngle() { return Range.scale(potSensor.getVoltage(), 0, potSensor.getMaxVoltage(), 0, 270); }

    //TODO: Sample classes for checking the color Red from the color sensor
    //public int getAmountRed() {return colorSensor.red();}
    //public int getAmountGreen() {return colorSensor.green();}
    //public int getAmountBlue() {return colorSensor.blue();}

    //TODO: Sample class for measuring distance from the distance sensor
    //public double getDistance(DistanceUnit du) { return distanceSensor.getDistance(du); }

    //TODO: Sample class for measuring the REV Control Hub heading through the IMU or navX
    public double getHeading(AngleUnit angleUnit) {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES); // if using the imu
        // return -navx_device.getYaw();  // if using the navX
    }

}
