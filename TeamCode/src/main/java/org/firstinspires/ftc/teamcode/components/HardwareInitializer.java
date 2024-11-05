package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;
import java.util.Map;

public class HardwareInitializer {

    private Map<String, RevBlinkinLedDriver> ledDrivers = new HashMap<>();
    private Map<String, DcMotor> motors = new HashMap<>();
    private Map<String, Servo> servos = new HashMap<>();


    // Method to initialize all hardware components
    public void initHardware(HardwareMap hardwareMap) {
//        ledDrivers.put("L_Light", hardwareMap.get(RevBlinkinLedDriver.class, "L_Light"));

        motors.put("FR", hardwareMap.get(DcMotor.class, "FR"));
        motors.put("FL", hardwareMap.get(DcMotor.class, "FL"));
        motors.put("BR", hardwareMap.get(DcMotor.class, "BR"));
        motors.put("BL", hardwareMap.get(DcMotor.class, "BL"));

        motors.put("SLIDESF", hardwareMap.get(DcMotor.class, "SLIDESF"));
        motors.put("SLIDESB", hardwareMap.get(DcMotor.class, "SLIDESB"));
        motors.put("ARM", hardwareMap.get(DcMotor.class, "ARM"));

        servos.put("ROTATE", hardwareMap.get(Servo.class, "ROTATE"));
        servos.put("PINCH", hardwareMap.get(Servo.class, "PINCH"));
        servos.put("WRISTL", hardwareMap.get(Servo.class, "WRISTL"));
        servos.put("WRISTR", hardwareMap.get(Servo.class, "WRISTR"));


    }

    // Getter for the LED lights by name
    public RevBlinkinLedDriver getLedDriver(String name) {
        return ledDrivers.get(name);
    }

    // Getter for the motors by name
    public DcMotor getMotor(String name) {
        return motors.get(name);
    }

    // Getter for servos by name
    public Servo getServo(String name) {
        return servos.get(name);
    }

}
