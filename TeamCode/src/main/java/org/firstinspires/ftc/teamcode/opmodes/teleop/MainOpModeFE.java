package org.firstinspires.ftc.teamcode.opmodes.teleop;

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
import org.firstinspires.ftc.teamcode.components.Values;

@TeleOp(group = "Actual", name = "MainOpModeFE")
public class MainOpModeFE extends OpMode {
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
    private int autoStep = 1;
    boolean pBasket = false;
    boolean pHome = false;
    boolean pRung = false;
    boolean pSubmersible = false;
    boolean pObservation = false;
    boolean pDropoff = false;

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

        mecanumDrive = new MecanumDriveFE(FR, FL, BR, BL, imu, gamepad1);
        claw = new ClawControl(WRIST_L, WRIST_R, ROTATE, PINCH, gamepad1, gamepad2, telemetry);
        arm = new ArmControl(SLIDES_F, SLIDES_B, ARM, gamepad1, gamepad2, telemetry);
        claw.init();
        arm.init();
        mecanumDrive.init();
    }

    @Override
    public void init_loop() {
        arm.initPosition();
        arm.telemetry();
    }

    @Override
    public void start() {
        claw.wristTarget = Values.WRIST_HOME;
    }

    @Override
    public void loop() {
        mecanumDrive.move();
        arm.move();
        claw.move(arm.slidesPosition);


        // Auto Stuff
        boolean basket = gamepad1.b;
        boolean home = gamepad1.ps;
        boolean submersible = gamepad2.a;
        boolean rung = gamepad1.y;
        boolean observation = gamepad1.a;
        boolean dropoff = gamepad2.y;

        if (observation && !pObservation && autoStep == 1) {
            autoStep = 1;
            autoProcess = "observation";
        }

        if (basket && !pBasket && autoStep == 1) {
            autoStep = 1;
            autoProcess = "basket";
        }

        if (submersible && !pSubmersible && autoStep == 1) {
            autoStep = 1;
            autoProcess = "submersible";
        }

        if (rung && !pRung && autoStep == 1) {
            autoStep = 1;
            autoProcess = "rung";
        }

        if (dropoff && !pDropoff && autoStep == 1) {
            autoStep = 1;
            autoProcess = "dropoff";
        }

        if (home && !pHome) {
            autoStep = 1;
            autoProcess = "home";
        }

        // Finite State Machine
        switch (autoProcess) {
            case "home":
                switch (autoStep) {
                    case 1:
                        if(home && !pHome && autoTime.milliseconds() > 100) {
                            arm.aMinSpeed = -0.8;
                            claw.rotateTarget = Values.ROTATE_INIT;
                            claw.wristTarget = Values.WRIST_HOME;
                            claw.clawClosed = true;
                            autoTime.reset();
                            autoStep = 10001;
                        }
                        break;
                    case 1000101:
                        arm.aMinSpeed = -0.8;
                        claw.rotateTarget = Values.ROTATE_INIT;
                        claw.wristTarget = Values.WRIST_HOME;
                        claw.clawClosed = true;
                        autoTime.reset();
                        autoStep = 10001;
                        break;
                    case 10001:
                        if(autoTime.milliseconds() > 300) {
                            if(arm.armPosition > 1150 && arm.slidesPosition > 400) {
                                arm.armTarget = arm.armPosition - 70;
                            } else if(arm.armPosition < 50) {
                                arm.armTarget = arm.armPosition + 70;
                            }
                            autoTime.reset();
                            autoStep += 1;
                        }
                        break;
                    case 10002:
                        if(autoTime.milliseconds() > 100) {
                            arm.slidesTarget = Values.SLIDES_HOME;
                            autoTime.reset();
                            autoStep += 1;
                        }
                        break;
                    case 10003:
                        if(autoTime.milliseconds() > 500) {
                            arm.aMinSpeed = -0.5;
                            if(arm.armPosition < 500) {
                                arm.aMinSpeed = -0.08;
                            }
                            arm.armTarget = Values.ARM_HOME;
                            autoTime.reset();
                            autoStep = 1;
                            autoProcess = "none";
                        }
                        break;
                }
                break;
            case "submersible":
                switch (autoStep) {
                    case 1:
                        if(submersible && !pSubmersible && autoTime.milliseconds() > 100) {
                            arm.aMinSpeed = -0.8;
                            claw.clawClosed = false;
                            arm.armTarget = Values.ARM_SUB;
                            autoTime.reset();
                            autoStep = 10001;
                        }
                        break;
                    case 10001:
                        if(autoTime.milliseconds() > 100) {
                            claw.wristTarget = Values.WRIST_HOME;
                            autoTime.reset();
                            autoStep += 1;
                        }
                        break;
                    case 10002:
                        if (autoTime.milliseconds() > 100) {
                            arm.slidesTarget = Values.SLIDES_SUB;
                            autoTime.reset();
                            autoStep = 2;
                        }
                        break;
                    case 2:
                        if(submersible && !pSubmersible && autoTime.milliseconds() > 100) {
                            claw.clawClosed = true;
                            autoTime.reset();
                            autoStep = 20001;
                        }
                        break;
                    case 20001:
                        if(autoTime.milliseconds() > 100) {
                            claw.rotateTarget = Values.ROTATE_INIT;
                            claw.wristTarget = Values.WRIST_SUB;
                            autoTime.reset();
                            autoStep += 1;
                        }
                        break;
                    case 20002:
                        if(autoTime.milliseconds() > 100) {
                            arm.slidesTarget = Values.SLIDES_HOME;
                            autoTime.reset();
                            autoStep = 3;
                        }
                        break;
                    case 3:
                        if(autoTime.milliseconds() > 1000) {
                            autoTime.reset();
                            autoStep = 1000101;
                            autoProcess = "home";
                        }
                        break;
                }
                break;
            case "basket":
                switch (autoStep) {
                    case 1:
                        if(basket && !pBasket && autoTime.milliseconds() > 100) {
                            arm.aMinSpeed = -0.5;
                            arm.armTarget = Values.ARM_HBASKET;
                            claw.wristPosition = Values.WRIST_HOME;
                            autoTime.reset();
                            autoStep += 1;
                        }

                        break;
                    case 2:
                        if(basket && !pBasket && autoTime.milliseconds() > 100) {
                            arm.slidesTarget = Values.SLIDES_HBASKET;
                            autoTime.reset();
                            autoStep += 1;
                        }
                        break;
                    case 3:
                        if(basket && !pBasket && autoTime.milliseconds() > 100) {
                            claw.wristTarget = Values.WRIST_MAX;
                            autoTime.reset();
                            autoStep += 1;
                        }
                        break;
                    case 4:
                        if(basket && !pBasket && autoTime.milliseconds() > 100) {
                            claw.clawClosed = false;
                            autoTime.reset();
                            autoStep = 40001;
                        }
                        break;
                    case 40001:
                        if(autoTime.milliseconds() > 100) {
                            claw.wristTarget = Values.WRIST_MED;
                            autoTime.reset();
                            autoStep += 1;
                        }
                        break;
                    case 40002:
                        if(autoTime.milliseconds() > 400) {
                            arm.sMinSpeed = -0.6;
                            arm.slidesTarget = Values.SLIDES_HOME;
                            claw.rotateTarget = Values.ROTATE_INIT;
                            autoStep += 1;
                            autoTime.reset();
                        }
                        break;
                    case 40003:
                        if (autoTime.milliseconds() > 500) {
                            arm.armTarget = Values.ARM_HOME;
                            claw.wristTarget = Values.WRIST_MAX;
                            autoStep += 1;
                            autoTime.reset();
                        }
                        break;
                    case 40004:
                        if (autoTime.milliseconds() > 500) {
                            claw.wristTarget = Values.WRIST_HOME;
                            autoStep = 1;
                            autoProcess = "none";
                        }
                        break;
                }
                break;
            case "rung":
                switch (autoStep) {
                    case 1:
                        if(rung && !pRung && autoTime.milliseconds() > 100) {
                            arm.aMinSpeed = -0.7;
                            claw.wristTarget = Values.WRIST_MAX;
                            arm.armTarget = Values.ARM_HRUNG;
                            autoTime.reset();
                            autoStep = 10001;
                        }
                        break;
                    case 10001:
                        if(autoTime.milliseconds() > 100) {
                            arm.slidesTarget = Values.SLIDES_HRUNG;
                            autoTime.reset();
                            autoStep = 2;
                        }
                        break;
                    case 2:
                        if(rung && !pRung && autoTime.milliseconds() > 100) {
                            arm.slidesTarget = Values.SLIDES_HRUNG_S;
                            autoTime.reset();
                            autoStep += 1;
                        }
                        break;
                    case 3:
                        if(rung && !pRung && autoTime.milliseconds() > 100) {
                            claw.wristTarget = Values.WRIST_HRUNG_S;
                            autoTime.reset();
                            autoStep += 1;
                        }
                        break;
                    case 4:
                        if(rung && !pRung && autoTime.milliseconds() > 100) {
                            claw.clawClosed = false;
                            autoTime.reset();
                            autoStep = 40001;
                        }
                        break;
                    case 40001:
                        if(autoTime.milliseconds() > 100) {
                            claw.wristTarget = Values.WRIST_MED;
                            autoTime.reset();
                            autoStep += 1;
                        }
                        break;
                    case 40002:
                        if(autoTime.milliseconds() > 400) {
                            arm.slidesTarget = Values.SLIDES_HOME;
                            claw.rotateTarget = Values.ROTATE_INIT;
                            autoStep += 1;
                            autoTime.reset();
                        }
                        break;
                    case 40003:
                        if (autoTime.milliseconds() > 500) {
                            arm.armTarget = Values.ARM_HOME;
                            claw.wristTarget = Values.WRIST_MAX;
                            autoStep += 1;
                            autoTime.reset();
                        }
                        break;
                    case 40004:
                        if (autoTime.milliseconds() > 500) {
                            claw.wristTarget = Values.WRIST_HOME;
                            autoStep = 1;
                            autoProcess = "none";
                        }
                        break;
                }
                break;
            case "observation":
                switch (autoStep) {
                    case 1:
                        if (observation && !pObservation && autoTime.milliseconds() > 100) {
                            claw.clawClosed = false;
                            arm.armTarget = Values.ARM_WALL;
                            arm.slidesTarget = Values.SLIDES_WALL;
                            claw.wristTarget = Values.WRIST_WALL;
                            autoTime.reset();
                            autoStep += 1;
                        }
                        break;
                    case 2:
                        if (observation && !pObservation && autoTime.milliseconds() > 100) {
                            claw.clawClosed = true;
                            autoTime.reset();
                            autoStep = 20001;
                        }
                        break;
                    case 20001:
                        if (autoTime.milliseconds() > 100) {
                            claw.wristTarget = Values.WRIST_WALL_UP;
                            autoTime.reset();
                            autoStep += 1;
                        }
                        break;
                    case 20002:
                        if (autoTime.milliseconds() > 100) {
                            arm.armTarget = arm.armPosition + 100;
                            autoTime.reset();
                            autoStep += 1;
                        }
                        break;
                    case 20003:
                        if (autoTime.milliseconds() > 200) {
                            arm.slidesTarget = Values.SLIDES_HOME;
                            autoTime.reset();
                            autoStep += 1;
                        }
                        break;
                    case 20004:
                        if (autoTime.milliseconds() > 300) {
                            claw.wristTarget = Values.WRIST_HOME;
                            autoTime.reset();
                            autoStep = 1;
                            autoProcess = "rung";
                        }
                        break;
                }
                break;
            case "dropoff":
                switch (autoStep) {
                    case 1:
                        if (dropoff && !pDropoff) {
                            arm.slidesTarget = Values.SLIDES_DROPOFF;
                            claw.wristTarget = Values.WRIST_MED;
                            autoTime.reset();
                            autoStep += 1;
                        }
                        break;
                    case 2:
                        if (dropoff && !pDropoff && autoTime.milliseconds() > 100) {
                            claw.clawClosed = false;
                            autoTime.reset();
                            autoStep += 1;
                        }
                        break;
                    case 3:
                        if (dropoff && !pDropoff && autoTime.milliseconds() > 100) {
                            arm.slidesTarget = Values.SLIDES_HOME;
                            autoTime.reset();
                            autoStep = 30001;
                        }
                        break;
                    case 30001:
                        if(autoTime.milliseconds() > 100) {
                            autoStep = 1000101;
                            autoProcess = "home";
                        }
                        break;
                }
                break;

        }

        pBasket = basket;
        pRung = rung;
        pSubmersible = submersible;
        pHome = home;
        pObservation = observation;
        pDropoff = dropoff;

        telemetry.addData("AutoProcess", autoProcess);
        telemetry.addData("AutoStep", autoStep);
        double heading = mecanumDrive.getHeading();
        telemetry.addData("Heading (Degrees)", heading);

        arm.telemetry();
        claw.telemetry();
    }
}
