package org.firstinspires.ftc.teamcode.opmodes.debugging;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.components.HardwareInitializer;
import org.firstinspires.ftc.teamcode.components.LightControl;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

@TeleOp(group = "Debugging", name = "Lighting")
public class Lighting extends OpMode {
    private HardwareInitializer hardwareInitializer;
    private RevBlinkinLedDriver L_Light;


    @Override
    public void init() {
        // Initialize and set hardware variables
        hardwareInitializer = new HardwareInitializer();
        hardwareInitializer.initHardware(hardwareMap);
        L_Light = hardwareInitializer.getLedDriver("L_Light");
        LightControl.setLightPattern(L_Light, "ORANGE");
    }

    @Override
    public void start() {
        LightControl.setLightPattern(L_Light, "RUNNING");
    }

    @Override
    public void loop() {

    }
}
