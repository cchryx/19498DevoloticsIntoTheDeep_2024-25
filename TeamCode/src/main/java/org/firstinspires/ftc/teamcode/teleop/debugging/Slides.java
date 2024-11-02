package org.firstinspires.ftc.teamcode.teleop.debugging;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.components.HardwareInitializer;
import org.firstinspires.ftc.teamcode.components.LinearSlides;

import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group = "Debugging", name = "LinearSlides")
public class Slides extends OpMode {

    private HardwareInitializer hardwareInitializer;
    private LinearSlides linearSlides;

    // Initialize hardware
    DcMotor SLIDES_F;
    DcMotor SLIDES_B;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize hardware
        hardwareInitializer = new HardwareInitializer();
        hardwareInitializer.initHardware(hardwareMap);

        SLIDES_F = hardwareInitializer.getMotor("SLIDESF");
        SLIDES_B = hardwareInitializer.getMotor("SLIDESB");

        linearSlides = new LinearSlides(SLIDES_F, SLIDES_B, gamepad1);
        linearSlides.initSlides();
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        linearSlides.moveSlides();
        telemetry.addData("slidesTarget", linearSlides.slidesTarget);
        telemetry.addData("slidesPosition", linearSlides.slidesPostition);
    }
}
