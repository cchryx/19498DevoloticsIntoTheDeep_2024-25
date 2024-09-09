package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.HashMap;
import java.util.Map;

public class HardwareInitializer {

    private Map<String, RevBlinkinLedDriver> ledDrivers = new HashMap<>();
    private Map<String, DcMotor> motors = new HashMap<>();

    // Method to initialize all hardware components
    public void initHardware(HardwareMap hardwareMap) {
        ledDrivers.put("L_Light", hardwareMap.get(RevBlinkinLedDriver.class, "L_Light"));
//        motors.put("leftMotor", hardwareMap.get(DcMotor.class, "left_motor"));
//        motors.put("rightMotor", hardwareMap.get(DcMotor.class, "right_motor"));
    }

    // Getter for the LED lights by name
    public RevBlinkinLedDriver getLedDriver(String name) {
        return ledDrivers.get(name);
    }

    // Getter for the motors by name
    public DcMotor getMotor(String name) {
        return motors.get(name);
    }
}
