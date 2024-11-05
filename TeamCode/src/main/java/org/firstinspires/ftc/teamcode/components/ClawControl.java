package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Values;

public class ClawControl {

    public Servo WRIST_L, WRIST_R, ROTATE, PINCH;
    public Gamepad gamepad1;

    // Constructor to initialize the motors and gamepad
    public ClawControl(Servo WRIST_L, Servo WRIST_R, Servo ROTATE, Servo PINCH, Gamepad gamepad1) {
        this.WRIST_L = WRIST_L;
        this.WRIST_R = WRIST_R;
        this.ROTATE = ROTATE;
        this.PINCH = PINCH;
        this.gamepad1 = gamepad1;

        this.WRIST_L.setPosition(Values.WRISTL_INIT);
        this.WRIST_R.setPosition(Values.WRISTR_INIT);
        this.ROTATE.setPosition(Values.ROTATE_INIT);
        this.PINCH.setPosition(Values.PINCH_MIN);
    }

    public void Claw_move() {

    }
}
