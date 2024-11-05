package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.Values;


//@Config // TODO: COMMENT OUT AFTER TUNING (http://192.168.43.1:8080/dash)
public class LinearSlides {

    private final double SLOWDOWN_FACTOR = 0.5;
    private DcMotor SLIDES_F, SLIDES_B;
    private Gamepad gamepad1;
    public PIDController slidesController;
    public double P = 0.01088, I = 0.000001, D = 0.00006, F = 0; // TODO: RM STATIC AFTER TUNING (http://192.168.43.1:8080/dash)
    public int slidesTarget = 0; // TODO: RM STATIC AFTER TUNING (http://192.168.43.1:8080/dash)
    public int slidesPostition;
    public double motorPowerSlides;
    public double slidesPID;

    // Constructor to initialize the motors and gamepad
    public LinearSlides(DcMotor SLIDES_F, DcMotor SLIDES_B, Gamepad gamepad1) {
        this.SLIDES_F = SLIDES_F;
        this.SLIDES_B = SLIDES_B;
        this.gamepad1 = gamepad1;
    }

    public void initSlides() {
        // Set SLIDES_B to reverse direction
        SLIDES_B.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set the motors to use encoders
        SLIDES_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SLIDES_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SLIDES_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SLIDES_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slidesController = new PIDController(P, I, D);
    }

    public void moveSlides() {
        /////////
        // PID //
        /////////

//        slidesController.setPID(P, I, D); // TODO: COMMENT OUT AFTER TUNING (http://192.168.43.1:8080/dash)
        int averageRawCurrentPos = (SLIDES_F.getCurrentPosition() + SLIDES_B.getCurrentPosition()) / 2;
        slidesPostition = averageRawCurrentPos;
        slidesPID = slidesController.calculate(slidesPostition, slidesTarget);
        motorPowerSlides = slidesPID + F;
        motorPowerSlides = Math.min(1, Math.max(-0.6, motorPowerSlides));
        SLIDES_F.setPower(motorPowerSlides);
        SLIDES_B.setPower(motorPowerSlides);

        ///////////////////////
        // CONTROLLER INPUTS //
        ///////////////////////

        // Manual Positioning
        if (gamepad1.right_bumper) {
            if(slidesTarget + Values.SLDS_INCR > Values.SLDS_MAX_EXTEND) {
                slidesTarget = Values.SLDS_MAX_EXTEND;
            } else {
                slidesTarget += Values.SLDS_INCR;
            }
        } else if (gamepad1.left_bumper) {
            if(slidesTarget - Values.SLDS_INCR < Values.SLDS_HOME) {
                slidesTarget = Values.SLDS_HOME;
            } else {
                slidesTarget -= Values.SLDS_INCR;
            }
        }

        // Slides Target Min/Max
        slidesTarget = Math.min(slidesTarget, Math.max(-Values.SLDS_INCR, slidesTarget));

        // Reset Positioning
        if(gamepad1.dpad_up) {
            slidesTarget = Values.SLDS_MAX_EXTEND;
        } else if (gamepad1.dpad_down) {
            slidesTarget = Values.SLDS_HOME;
        }

        // Finite State Machine


    }
}
