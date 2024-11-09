package org.firstinspires.ftc.teamcode.teleop.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.ArmControl;
import org.firstinspires.ftc.teamcode.components.ClawControl;
import org.firstinspires.ftc.teamcode.components.HardwareInitializer;
import org.firstinspires.ftc.teamcode.components.MecanumDriveFE;
import org.firstinspires.ftc.teamcode.util.Values;

@TeleOp(group = "Actual", name = "MainOpMode")
public class MainOpMode extends OpMode {
    private HardwareInitializer hardwareInitializer;
    private ArmControl arm;
    private ClawControl claw;
    private MecanumDriveFE mecanumDrive;

    // Initialize hardware
    DcMotor SLIDES_F;
    DcMotor SLIDES_B;
    DcMotor ARM;
    Servo ROTATE;
    Servo PINCH;
    Servo WRIST_L;
    Servo WRIST_R;
    DcMotor FR;
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;
    IMU imu;

    private ElapsedTime autoTime = new ElapsedTime();
    private String autoProcess = "none";
    private int autoStep = 0;
    boolean pBasket = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize hardware
        hardwareInitializer = new HardwareInitializer();
        hardwareInitializer.initHardware(hardwareMap);

        SLIDES_F = hardwareInitializer.getMotor("SLIDESF");
        SLIDES_B = hardwareInitializer.getMotor("SLIDESB");
        ARM = hardwareInitializer.getMotor("ARM");
        ROTATE = hardwareInitializer.getServo("ROTATE");
        PINCH = hardwareInitializer.getServo("PINCH");
        WRIST_L = hardwareInitializer.getServo("WRISTL");
        WRIST_R = hardwareInitializer.getServo("WRISTR");
        FR = hardwareInitializer.getMotor("FR");
        FL = hardwareInitializer.getMotor("FL");
        BR = hardwareInitializer.getMotor("BR");
        BL = hardwareInitializer.getMotor("BL");
        imu = hardwareMap.get(IMU.class, "imu");

        // Set the IMU parameters
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        imu.resetYaw();

        // Create a new MecanumDriveFE object
        mecanumDrive = new MecanumDriveFE(FR, FL, BR, BL, imu, gamepad1, gamepad2);
        claw = new ClawControl(WRIST_L, WRIST_R, ROTATE, PINCH, gamepad1, gamepad2, telemetry);
        arm = new ArmControl(SLIDES_F, SLIDES_B, ARM, gamepad1, gamepad2, telemetry);
        claw.init();
        arm.init();
        mecanumDrive.init();
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        mecanumDrive.move();
        arm.move();
        claw.move(arm.slidesPosition);


        // Auto Stuff
        boolean basket = gamepad1.b;
        if (basket && !pBasket && autoStep == 0) {
            autoStep = 1;
            autoProcess = "basket";
        }

        // Finite State Machine
        switch (autoProcess) {
            case "none":
                break;
            case "basket":
                switch (autoStep) {
                    case 1:
                        if(basket && !pBasket && autoTime.milliseconds() > 100) {
                            arm.minSpeed = -0.3;
                            claw.clawClosed = false;
                            arm.armTarget = 70;
                            autoTime.reset();
                            autoStep = 10001;
                        }
                        break;
                    case 10001:
                        if (autoTime.milliseconds() > 200) {
                            arm.slidesTarget = Values.SLIDES_SUB;
                            autoTime.reset();
                            autoStep = 2;
                        }
                        break;
                    case 2:
                        if(basket && !pBasket && autoTime.milliseconds() > 100) {
                            arm.armTarget = 100;
                            autoTime.reset();
                            autoStep = 20001;
                        }
                        break;
                    case 20001:
                        if (basket && !pBasket &&autoTime.milliseconds() > 100) {
                            claw.clawClosed = true;
                            autoTime.reset();
                            autoStep += 1;
                        }
                        break;
                    case 20002:
                        if (arm.armPosition <= 400) {
                            claw.rotateTarget = Values.ROTATE_INIT;
                            arm.slidesTarget = Values.SLIDES_HOME;
                            autoTime.reset();
                            autoStep = 3;
                        }
                        break;
                    case 3:
                        if(basket && !pBasket && autoTime.milliseconds() > 100) {
                            arm.armTarget = Values.ARM_HBASKET;
                            autoTime.reset();
                            autoStep += 1;
                        }

                        break;
                    case 4:
                        if(basket && !pBasket && autoTime.milliseconds() > 100) {
                            arm.slidesTarget = Values.SLIDES_HBASKET;
                            autoTime.reset();
                            autoStep = 40001;
                        }
                        break;
                    case 40001:
                        if(basket && !pBasket && autoTime.milliseconds() > 100) {
                            claw.wristTarget = Values.WRIST_MAX;
                            autoTime.reset();
                            autoStep = 5;
                        }
                        break;
                    case 5:
                        if(basket && !pBasket && autoTime.milliseconds() > 100) {
                            claw.clawClosed = false;
                            autoTime.reset();
                            autoStep = 50001;
                        }
                        break;
                    case 50001:
                        if(autoTime.milliseconds() > 100) {
                            claw.wristTarget = Values.WRIST_HOME;
                            autoTime.reset();
                            autoStep = 6;
                        }
                        break;
                    case 6:
                        if (basket && !pBasket && autoTime.milliseconds() > 100) {
                            arm.slidesTarget = Values.SLIDES_HOME;
                            autoTime.reset();
                            autoStep += 1;
                        }
                        break;
                    case 7:
                        if (basket && !pBasket && autoTime.milliseconds() > 100) {
                            arm.minSpeed = -0.03;
                            arm.armTarget = Values.ARM_MIN;
                            autoTime.reset();
                            autoStep = 0;
                        }
                        break;
                }
                break;
        }

        pBasket = basket;

        telemetry.addData("AutoProcess", autoProcess);
        telemetry.addData("AutoStep", autoStep);
        double heading = mecanumDrive.getHeading();
        telemetry.addData("Heading (Degrees)", heading);

        arm.telemetry();
        claw.telemetry();
    }
}
