package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareInitializer {

    private RevBlinkinLedDriver L_Light;

    // Method to initialize all hardware components
    public void initHardware(HardwareMap hardwareMap) {
        L_Light = hardwareMap.get(RevBlinkinLedDriver.class, "L_Light");
    }

    // Getter for the LED light
    public RevBlinkinLedDriver getL_Light() {
        return L_Light;
    }
}
