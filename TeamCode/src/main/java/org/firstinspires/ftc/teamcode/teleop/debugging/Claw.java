package org.firstinspires.ftc.teamcode.teleop.debugging;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.ClawControl;
import org.firstinspires.ftc.teamcode.components.HardwareInitializer;
import org.firstinspires.ftc.teamcode.util.Values;

import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(group = "Debugging", name = "Claw")
public class Claw extends OpMode {

    private HardwareInitializer hardwareInitializer;
    private ClawControl claw;

    // Initialize hardware
    Servo ROTATE;
    Servo PINCH;
    Servo WRIST_L;
    Servo WRIST_R;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize hardware
        hardwareInitializer = new HardwareInitializer();
        hardwareInitializer.initHardware(hardwareMap);

        ROTATE = hardwareInitializer.getServo("ROTATE");
        PINCH = hardwareInitializer.getServo("PINCH");
        WRIST_L = hardwareInitializer.getServo("WRISTL");
        WRIST_R = hardwareInitializer.getServo("WRISTR");

        claw = new ClawControl(WRIST_L, WRIST_R, ROTATE, PINCH, gamepad1, gamepad2, telemetry);
        claw.init();
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
//        claw.move();
        claw.telemetry();
    }
}
