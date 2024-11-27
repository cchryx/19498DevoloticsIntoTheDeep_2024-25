package org.firstinspires.ftc.teamcode.opmodes.debugging;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.components.HardwareInitializer;
import org.firstinspires.ftc.teamcode.components.ArmControl;

import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group = "Debugging", name = "Arm")
public class Arm extends OpMode {

    private HardwareInitializer hardwareInitializer;
    private ArmControl arm;

    // Initialize hardware
    DcMotor SLIDES_F;
    DcMotor SLIDES_B;
    DcMotor ARM;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize hardware
        hardwareInitializer = new HardwareInitializer();
        hardwareInitializer.initHardware(hardwareMap);

        SLIDES_F = hardwareInitializer.getMotor("SLIDESF");
        SLIDES_B = hardwareInitializer.getMotor("SLIDESB");
        ARM = hardwareInitializer.getMotor("ARM");

        arm = new ArmControl(SLIDES_F, SLIDES_B, ARM, gamepad1, gamepad2, telemetry);
        arm.init();
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        arm.move();
        arm.telemetry();
    }
}
