package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Values;

@Config // TODO: COMMENT OUT AFTER TUNING (http://192.168.43.1:8080/dash)
public class ClawControl {

    public Servo WRIST_L, WRIST_R, ROTATE, PINCH;
    public Gamepad gamepad1, gamepad2;
    public Telemetry telemetry;

    public boolean clawClosed = false;
    public boolean clawHold = false;

    public static double wristTarget = 0, rotateTarget = Values.ROTATE_INIT; // TODO: RM STATIC AFTER TUNING (http://192.168.43.1:8080/dash)
    public double wristPosition = 0;

    // Constructor to initialize the motors and gamepad
    public ClawControl(
            Servo WRIST_L,
            Servo WRIST_R,
            Servo ROTATE,
            Servo PINCH,
            Gamepad gamepad1,
            Gamepad gamepad2,
            Telemetry telemetry
    ) {
        this.WRIST_L = WRIST_L;
        this.WRIST_R = WRIST_R;
        this.ROTATE = ROTATE;
        this.PINCH = PINCH;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;

    }

    public void init() {
        // Set WRIST_L to reverse direction
        WRIST_R.setDirection(Servo.Direction.REVERSE);

        WRIST_L.setPosition(Values.WRIST_HOME + Values.WRIST_OFFSET);
        WRIST_R.setPosition(Values.WRIST_HOME);
        ROTATE.setPosition(Values.ROTATE_INIT);
        PINCH.setPosition(Values.PINCH_MIN);
    }

    public void move(double slidesPosition) {
        // Manual Positioning Rotate
        if (gamepad2.right_bumper && slidesPosition > 50) {
            if(ROTATE.getPosition() + Values.ROTATE_INCR > Values.ROTATE_R_MAX) {
                rotateTarget = Values.ROTATE_R_MAX;
            } else {
                rotateTarget += Values.ROTATE_INCR;
            }
        } else if (gamepad2.left_bumper && slidesPosition > 50) {
            if(ROTATE.getPosition() - Values.ROTATE_INCR < Values.ROTATE_L_MAX) {
                rotateTarget = Values.ROTATE_L_MAX;
            } else {
                rotateTarget -= Values.ROTATE_INCR;
            }
        }

        if (gamepad2.right_stick_y > 1 && slidesPosition > 50) {
            double wristIncr = Values.WRIST_INCR * gamepad2.right_stick_y;
            double newWristPos = WRIST_R.getPosition() + wristIncr;
            if(newWristPos > Values.WRIST_MAX) {
                wristTarget = Values.WRIST_MAX;
            } else {
                wristTarget += wristIncr;
            }
        } else if (gamepad2.right_stick_y > 1 && slidesPosition > 50) {
            double wristIncr = Values.WRIST_INCR * gamepad2.right_stick_y;
            double newWristPos = WRIST_R.getPosition() - wristIncr;
            if(newWristPos < Values.WRIST_MIN) {
                wristTarget = Values.WRIST_MIN;
            } else {
                wristTarget -= wristIncr;
            }
        }

        if (gamepad2.dpad_left && slidesPosition > 50) {
            rotateTarget = Values.ROTATE_L_MAX;
        } else if (gamepad2.dpad_right && slidesPosition > 50) {
            rotateTarget = Values.ROTATE_R_MAX;
        } else if (gamepad2.dpad_up) {
            rotateTarget = Values.ROTATE_INIT;
        }


        if(gamepad2.x){
            if (!clawHold) {
                clawHold = true;
                clawClosed = !clawClosed;
            }
        } else {
            clawHold = false;
        }

        if (!clawClosed){
            PINCH.setPosition(Values.PINCH_MAX);
        } else{
            PINCH.setPosition(Values.PINCH_MIN);
        }
        WRIST_R.setPosition(wristTarget);
        WRIST_L.setPosition(wristTarget + Values.WRIST_OFFSET);
        ROTATE.setPosition(rotateTarget);

    }

    public void telemetry() {
        telemetry.addData("wristPosition", wristPosition);
    }
}
