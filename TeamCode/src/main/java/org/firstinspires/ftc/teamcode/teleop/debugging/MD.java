package org.firstinspires.ftc.teamcode.teleop.debugging;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.HardwareInitializer;
import org.firstinspires.ftc.teamcode.util.MecanumDrive;


@TeleOp(group = "Debugging", name = "MecanumDrive")
public class MD extends OpMode {

    private HardwareInitializer hardwareInitializer;
    private MecanumDrive mecanumDrive;

    DcMotor FR;
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;

    @Override
    public void init() {
        hardwareInitializer = new HardwareInitializer();
        hardwareInitializer.initHardware(hardwareMap);

        FR = hardwareInitializer.getMotor("FR");
        FL = hardwareInitializer.getMotor("FL");
        BR = hardwareInitializer.getMotor("BR");
        BL = hardwareInitializer.getMotor("BL");

        mecanumDrive = new MecanumDrive(FR, FL, BR, BL, gamepad1);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void loop() {
        mecanumDrive.MecanumDrive_move();
    }
}
