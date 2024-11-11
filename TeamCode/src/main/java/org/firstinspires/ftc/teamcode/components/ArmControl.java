package org.firstinspires.ftc.teamcode.components;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Values;

@Config // TODO: COMMENT OUT AFTER TUNING (http://192.168.43.1:8080/dash)
public class ArmControl {

    private final double SLOWDOWN_FACTOR = 0.5;
    private DcMotor SLIDES_F, SLIDES_B, ARM;
    private Gamepad gamepad1, gamepad2;
    Telemetry telemetry;
    public PIDController slidesController, armController;
    public static double sP = 0.01088, sI = 0.01088, sD = 0.00006, sF = 0; // TODO: RM STATIC AFTER TUNING (http://192.168.43.1:8080/dash)
    public static double aP = 0.025, aI = 0, aD = 0.00089, aF = 0.021; // TODO: RM STATIC AFTER TUNING (http://192.168.43.1:8080/dash)
    public static int slidesTarget, armTarget = 0; // TODO: RM STATIC AFTER TUNING (http://192.168.43.1:8080/dash)
    public int slidesPosition, armPosition = 0;
    public double motorPowerSlides, motorPowerArm;
    public double slidesPID, armPID;

    public double minSpeed = -0.8;

    // Constructor to initialize the motors and gamepad
    public ArmControl(DcMotor SLIDES_F, DcMotor SLIDES_B, DcMotor ARM, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        this.SLIDES_F = SLIDES_F;
        this.SLIDES_B = SLIDES_B;
        this.ARM = ARM;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
    }

    public void init() {
        SLIDES_F.setZeroPowerBehavior(BRAKE);
        SLIDES_B.setZeroPowerBehavior(BRAKE);
        ARM.setZeroPowerBehavior(BRAKE);

        // Set SLIDES_B to reverse direction
        SLIDES_B.setDirection(DcMotor.Direction.REVERSE);

        // Set ARM to reverse direction
        ARM.setDirection(DcMotor.Direction.REVERSE);

        // Set the motors to use encoders
        SLIDES_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SLIDES_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SLIDES_F.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SLIDES_B.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ARM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ARM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slidesController = new PIDController(sP, sI, sD);
        armController = new PIDController(aP, aI, aD);
    }

    public void move() {
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
        armController.setPID(aP, aI, aD); // TODO: COMMENT OUT AFTER TUNING (http://192.168.43.1:8080/dash)
        int averageRawCurrentPos_a = ARM.getCurrentPosition();
        armPosition = averageRawCurrentPos_a;
        armPID = armController.calculate(armPosition, armTarget);
        double aFeed = Math.cos(Math.toRadians((armTarget - 180) / Values.TICKS_PER_DEG_GOBUILDA)) * aF;
        motorPowerArm = armPID + aFeed;
        motorPowerArm = Math.min(0.8, Math.max(minSpeed, motorPowerArm));
        ARM.setPower(motorPowerArm);

        ///////////////////////
        // CONTROLLER INPUTS //
        ///////////////////////

        // Manual Positioning Slides
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

        // Manual Positioning Arm
        if (gamepad2.right_trigger > 0) {
            if(armPosition + Values.ARM_INCR > Values.ARM_MAX) {
                armTarget = Values.ARM_MAX;
            } else {
                armTarget += Values.ARM_INCR * gamepad2.right_trigger;
            }
        } else if (gamepad2.left_trigger > 0) {
            if(armPosition - Values.ARM_INCR < Values.ARM_MIN) {
                armTarget = Values.ARM_MIN;
            } else {
                armTarget -= Values.ARM_INCR * gamepad2.left_trigger;
            }
        }

    }

    public void telemetry() {
        telemetry.addData("slidesTarget", slidesTarget);
        telemetry.addData("slidesPosition", slidesPosition);
        telemetry.addData("armTarget", armTarget);
        telemetry.addData("armPosition", armPosition);
    }
}
