package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class MecanumDrive {

    public DcMotor FR, FL, BR, BL;
    public Gamepad gamepad1;

    // Constructor to initialize the motors and gamepad
    public MecanumDrive(DcMotor FR, DcMotor FL, DcMotor BR, DcMotor BL, Gamepad gamepad1) {
        this.FR = FR;
        this.FL = FL;
        this.BR = BR;
        this.BL = BL;
        this.gamepad1 = gamepad1;
    }

    // MecanumDrive_move method to handle movement
    public void MecanumDrive_move() {
        // Set controller input
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        // Calculate setting motor power
        double FR_power = rx + (-y - x) / denominator;
        double BR_power = rx + (-y + x) / denominator;
        double FL_power = rx + (y - x) / denominator;
        double BL_power = rx + (y + x) / denominator;

        // Set motor powers
        FR.setPower(FR_power);
        BR.setPower(BR_power);
        FL.setPower(FL_power);
        BL.setPower(BL_power);
    }
}
