package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.HardwareInitializer;
import org.firstinspires.ftc.teamcode.util.LightControl;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

@TeleOp(group = "Template", name = "Teleop Template")
public class teleopTemplate extends OpMode {
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
