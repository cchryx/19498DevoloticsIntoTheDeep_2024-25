package org.firstinspires.ftc.teamcode.teleop.debugging;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.components.HardwareInitializer;
import org.firstinspires.ftc.teamcode.components.MecanumDrive;


@TeleOp(group = "Debugging", name = "MecanumDrive")
public class MD extends OpMode {

    private HardwareInitializer hardwareInitializer;
    private MecanumDrive mecanumDrive;

    // Initialize hardware
    DcMotor FR;
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;

    @Override
    public void init() {
        // Initialize hardware
        hardwareInitializer = new HardwareInitializer();
        hardwareInitializer.initHardware(hardwareMap);

        FR = hardwareInitializer.getMotor("FR");
        FL = hardwareInitializer.getMotor("FL");
        BR = hardwareInitializer.getMotor("BR");
        BL = hardwareInitializer.getMotor("BL");

        // Create a new MecanumDrive object
        mecanumDrive = new MecanumDrive(FR, FL, BR, BL, gamepad1);
        mecanumDrive.init();
    }

    @Override
    public void loop() {
        // Call the move method to make the drivetrain move
        mecanumDrive.move();
    }
}
