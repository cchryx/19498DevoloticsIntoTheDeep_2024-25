package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config // TODO: COMMENT OUT AFTER TUNING (http://192.168.43.1:8080/dash)
public class ClawControl {

    public Servo WRIST_L, WRIST_R, ROTATE, PINCH;
    public Gamepad gamepad1, gamepad2;
    public Telemetry telemetry;

    public boolean clawClosed = true;
    public boolean pClaw = false;

    public static double wristTarget = Values.WRIST_MAX, rotateTarget = Values.ROTATE_INIT; // TODO: RM STATIC AFTER TUNING (http://192.168.43.1:8080/dash)
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

        WRIST_L.setPosition(Values.WRIST_MAX + Values.WRIST_OFFSET);
        WRIST_R.setPosition(Values.WRIST_MAX);
        ROTATE.setPosition(Values.ROTATE_INIT);
        PINCH.setPosition(Values.PINCH_MIN);
    }

    public void move(double slidesPosition) {

        rotateTarget = Math.min(1, Math.max(0, rotateTarget));

        if (slidesPosition < 50) {
            rotateTarget = Values.ROTATE_INIT;
        } else {
            rotateTarget += gamepad2.left_stick_x/100;
        }


//        if (gamepad2.left_stick_x > 0 && slidesPosition > 50) {
//            double rotateIncr = Values.ROTATE_INCR * gamepad2.left_stick_x;
//            double newRotatePos = ROTATE.getPosition() + rotateIncr;
//            rotateTarget += rotateIncr;
//
////
//        } else if (gamepad2.left_stick_x < 0 && slidesPosition > 50) {
//            double rotateIncr = Values.ROTATE_INCR * gamepad2.left_stick_x;
//            double newRotatePos = ROTATE.getPosition() + rotateIncr;
//            rotateTarget += rotateIncr;
//
////            if(newRotatePos < Values.ROTATE_L_MAX) {
////                rotateTarget = Values.ROTATE_L_MAX;
////            } else {
////                rotateTarget += rotateIncr;
////            }
//        }

        if (gamepad2.left_stick_y > 0) {
            double wristIncr = Values.WRIST_INCR * gamepad2.left_stick_y;
            double newWristPos = wristPosition + wristIncr;
            if(newWristPos > Values.WRIST_MAX) {
                wristTarget = Values.WRIST_MAX;
            } else {
                wristTarget = newWristPos;
            }
        } else if (gamepad2.left_stick_y < 0) {
            double wristIncr = Values.WRIST_INCR * gamepad2.left_stick_y;
            double newWristPos = wristPosition + wristIncr;
            if(newWristPos < Values.WRIST_MIN) {
                wristTarget = Values.WRIST_MIN;
            } else {
                wristTarget = newWristPos;
            }
        }

        if (gamepad2.dpad_left && slidesPosition > 50) {
            rotateTarget = Values.ROTATE_L_MAX;
        } else if (gamepad2.dpad_right && slidesPosition > 50) {
            rotateTarget = Values.ROTATE_R_MAX;
        } else if (gamepad2.dpad_up) {
            rotateTarget = Values.ROTATE_INIT;
        }

        boolean claw = gamepad2.x;
        if(!pClaw && claw) {
            clawClosed = !clawClosed;
        }
        pClaw = claw;

        if (!clawClosed){
            PINCH.setPosition(Values.PINCH_MAX);
        } else{
            PINCH.setPosition(Values.PINCH_MIN);
        }

        WRIST_R.setPosition(wristTarget);
        WRIST_L.setPosition(wristTarget + Values.WRIST_OFFSET);
        ROTATE.setPosition(rotateTarget);

        wristPosition = WRIST_R.getPosition();

    }

    public void telemetry() {
        telemetry.addData("wristPosition", wristPosition);
        telemetry.addData("clawHold", pClaw);
        telemetry.addData("clawClosed", clawClosed);


    }
}
