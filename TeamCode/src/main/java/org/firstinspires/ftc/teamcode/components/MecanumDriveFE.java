package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDriveFE {

    private DcMotor FR, FL, BR, BL;
    private IMU imu;
    private Gamepad gamepad1;

    // Constructor to initialize the motors and gamepad
    public MecanumDriveFE(DcMotor FR, DcMotor FL, DcMotor BR, DcMotor BL, IMU imu, Gamepad gamepad1) {
        this.FR = FR;
        this.FL = FL;
        this.BR = BR;
        this.BL = BL;
        this.imu = imu;
        this.gamepad1 = gamepad1;
    }

    // MecanumDrive_move method to handle movement
    public void MecanumDrive_move() {
        // Set controller input
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // For resetting the bot heading in the middle of a match
        if (gamepad1.options) {
            imu.resetYaw();
        }

        // Set bot heading
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Calculate rotation using bot heading
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        // Calculate setting motor power
        double FR_power = rx + (-rotY - rotX) / denominator;
        double BR_power = rx + (-rotY + rotX) / denominator;
        double FL_power = rx + (rotY - rotX) / denominator;
        double BL_power = rx + (rotY + rotX) / denominator;

        // Set motor powers
        FR.setPower(FR_power);
        BR.setPower(BR_power);
        FL.setPower(FL_power);
        BL.setPower(BL_power);
    }

    // Get the heading of the robot
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}
