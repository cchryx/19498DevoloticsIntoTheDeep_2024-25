package org.firstinspires.ftc.teamcode.teleop.debugging;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.components.HardwareInitializer;
import org.firstinspires.ftc.teamcode.components.MecanumDriveFE;

@TeleOp(group = "Debugging", name = "MecanumDriveFE")
public class MD_FE extends OpMode {

    private HardwareInitializer hardwareInitializer;
    private MecanumDriveFE mecanumDrive;

    // Initialize hardware
    DcMotor FR;
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;
    IMU imu;

    @Override
    public void init() {
        // Initialize hardware
        hardwareInitializer = new HardwareInitializer();
        hardwareInitializer.initHardware(hardwareMap);

        FR = hardwareInitializer.getMotor("FR");
        FL = hardwareInitializer.getMotor("FL");
        BR = hardwareInitializer.getMotor("BR");
        BL = hardwareInitializer.getMotor("BL");
        imu = hardwareMap.get(IMU.class, "imu");

        // Set the IMU parameters
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        // Create a new MecanumDriveFE object
        mecanumDrive = new MecanumDriveFE(FR, FL, BR, BL, imu, gamepad1);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void loop() {
        // Call the move method to make the drivetrain move
        mecanumDrive.MecanumDrive_move();
        // Retrieve and display the heading on telemetry
        double heading = mecanumDrive.getHeading();
        telemetry.addData("Heading (Degrees)", heading);
        telemetry.update();
    }
}
