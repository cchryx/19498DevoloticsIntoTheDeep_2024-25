package org.firstinspires.ftc.teamcode.components;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDriveFE {

    public DcMotor FR, FL, BR, BL;
    public IMU imu;
    public Gamepad gamepad1, gamepad2;

    public double FR_power, BR_power, FL_power, BL_power;


    // Constructor to initialize the motors and gamepad
    public MecanumDriveFE(DcMotor FR, DcMotor FL, DcMotor BR, DcMotor BL, IMU imu, Gamepad gamepad1) {
        this.FR = FR;
        this.FL = FL;
        this.BR = BR;
        this.BL = BL;
        this.imu = imu;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

    }

    public void init() {
        FR.setZeroPowerBehavior(BRAKE);
        FL.setZeroPowerBehavior(BRAKE);
        BR.setZeroPowerBehavior(BRAKE);
        BL.setZeroPowerBehavior(BRAKE);
        imu.resetYaw();
    }

    // MecanumDrive_move method to handle movement
    public void move() {
        // Set controller input
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // For resetting the bot heading in the middle of a match
        if (gamepad1.options) {
            imu.resetYaw();
        }

        // Set bot heading
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Calculate rotation using bot heading
        double rotX = x * Math.cos(botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        // Calculate setting motor power
        FR_power = rx + (-rotY - rotX) / denominator;
        BR_power = rx + (-rotY + rotX) / denominator;
        FL_power = rx + (rotY - rotX) / denominator;
        BL_power = rx + (rotY + rotX) / denominator;

        if(gamepad1.right_trigger > 0) {
            FR_power *= Values.DT_SLOW_FACTOR;
            BR_power *= Values.DT_SLOW_FACTOR;
            FL_power *= Values.DT_SLOW_FACTOR;
            BL_power *= Values.DT_SLOW_FACTOR;
        }

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
