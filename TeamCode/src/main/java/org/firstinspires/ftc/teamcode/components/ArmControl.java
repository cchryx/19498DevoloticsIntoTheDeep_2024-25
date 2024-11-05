package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.Values;

@Config // TODO: COMMENT OUT AFTER TUNING (http://192.168.43.1:8080/dash)
public class ArmControl {

    private final double SLOWDOWN_FACTOR = 0.5;
    private DcMotor SLIDES_F, SLIDES_B, ARM;
    private Gamepad gamepad1;
    public PIDController slidesController, armController;
    public static double sP = 0.01088, sI = 0.000001, sD = 0.00006, sF = 0; // TODO: RM STATIC AFTER TUNING (http://192.168.43.1:8080/dash)
    public static double aP = 0.01088, aI = 0.000001, aD = 0.00006, aF = 0; // TODO: RM STATIC AFTER TUNING (http://192.168.43.1:8080/dash)
    public static int slidesTarget, armTarget = 0; // TODO: RM STATIC AFTER TUNING (http://192.168.43.1:8080/dash)
    public static int slidesPosition, armPosition = 0;
    public double motorPowerSlides, motorPowerArm;
    public double slidesPID, armPID;

    // Constructor to initialize the motors and gamepad
    public ArmControl(DcMotor SLIDES_F, DcMotor SLIDES_B, DcMotor ARM, Gamepad gamepad1) {
        this.SLIDES_F = SLIDES_F;
        this.SLIDES_B = SLIDES_B;
        this.ARM = ARM;
        this.gamepad1 = gamepad1;
    }

    public void initArm() {
        // Set SLIDES_B to reverse direction
        SLIDES_B.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set ARM to reverse direction
        ARM.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set the motors to use encoders
        SLIDES_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SLIDES_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SLIDES_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SLIDES_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slidesController = new PIDController(sP, sI, sD);
        armController = new PIDController(aP, sI, sD);
    }

    public void moveArm() {
        //////////
        // PIDs //
        //////////

        // Slides
        slidesController.setPID(sP, sI, sD); // TODO: COMMENT OUT AFTER TUNING (http://192.168.43.1:8080/dash)
        int averageRawCurrentPos_s = (SLIDES_F.getCurrentPosition() + SLIDES_B.getCurrentPosition()) / 2;
        slidesPosition = averageRawCurrentPos_s;
        slidesPID = slidesController.calculate(slidesPosition, slidesTarget);
        motorPowerSlides = slidesPID + sF;
        motorPowerSlides = Math.min(1, Math.max(-0.6, motorPowerSlides));
        SLIDES_F.setPower(motorPowerSlides);
        SLIDES_B.setPower(motorPowerSlides);

        // Arm
        slidesController.setPIDF(aP, aI, aD, aF); // TODO: COMMENT OUT AFTER TUNING (http://192.168.43.1:8080/dash)
        int averageRawCurrentPos_a = ARM.getCurrentPosition();
        armPosition = averageRawCurrentPos_a;

        armPID = armController.calculate(armPosition, armTarget);
        motorPowerArm = armPID + aF;
        motorPowerArm = Math.min(1, Math.max(-0.6, motorPowerArm));
        ARM.setPower(motorPowerArm);

        ///////////////////////
        // CONTROLLER INPUTS //
        ///////////////////////

        // Manual Positioning
        if (gamepad1.right_bumper) {
            if(slidesTarget + Values.SLIDES_INCR > Values.SLIDES_MAX_EXTEND) {
                slidesTarget = Values.SLIDES_MAX_EXTEND;
            } else {
                slidesTarget += Values.SLIDES_INCR;
            }
        } else if (gamepad1.left_bumper) {
            if(slidesTarget - Values.SLIDES_INCR < Values.SLIDES_HOME) {
                slidesTarget = Values.SLIDES_HOME;
            } else {
                slidesTarget -= Values.SLIDES_INCR;
            }
        }

        // Slides Target Min/Max
        slidesTarget = Math.min(slidesTarget, Math.max(-Values.SLIDES_INCR, slidesTarget));

        // Reset Positioning
        if(gamepad1.dpad_up) {
            slidesTarget = Values.SLIDES_MAX_EXTEND;
        } else if (gamepad1.dpad_down) {
            slidesTarget = Values.SLIDES_HOME;
        }

        // Finite State Machine


    }
}
