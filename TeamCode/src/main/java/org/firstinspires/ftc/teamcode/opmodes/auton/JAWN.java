package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.components.ArmControl;
import org.firstinspires.ftc.teamcode.components.ClawControl;
import org.firstinspires.ftc.teamcode.components.HardwareInitializer;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class JAWN extends OpMode {
    private HardwareInitializer hardwareInitializer;
    private ArmControl arm;
    private ClawControl claw;
    private MecanumDrive drive;

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

    // Autonomous
    Pose2d START_POSE = new Pose2d(14, -57, Math.toRadians(90));
    TrajectoryActionBuilder tab1 = null;

    // Init Positions
    public final int  PINCHSERVO = 0, WRISTSERVO = 1, ROTATESERVO = 2;

    public List<int[]> PROGRAM = new ArrayList<>();

    // Miscellaneous
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime totalTime = new ElapsedTime();
    private ElapsedTime timeout = new ElapsedTime();
    public int prevLine = -1;
    public int line = 0;


    public int rPos = 0;
    public int wPos = 0;
    public int status = 0;

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

        claw = new ClawControl(WRIST_L, WRIST_R, ROTATE, PINCH, gamepad1, gamepad2, telemetry);
        arm = new ArmControl(SLIDES_F, SLIDES_B, ARM, gamepad1, gamepad2, telemetry);
        claw.init();
        arm.init();
    }

    @Override
    public void start() {
        totalTime.reset();
        telemetry.update();
        // Starting position
        drive = new MecanumDrive(hardwareMap, START_POSE);

        tab1 = drive.actionBuilder(START_POSE)
                .lineToY(-50);


        // Build Autonomous Program
        buildProgram();
    }

    @Override
    public void loop() {
        //region PID
        arm.move();
        claw.move(arm.slidesPosition);

        claw.clawClosed = true;
        claw.wristTarget = wPos;
        claw.rotateTarget = rPos;

        //PROGRAM
        if (line < PROGRAM.size()) {
            int func = PROGRAM.get(line)[0];
            int arg1 = PROGRAM.get(line)[1];
            int arg2 = PROGRAM.get(line)[2];
//            int arg3 = program.get(line)[3];

            boolean CHANGE_LINE = false;

            switch (func) {
                case 1:
                    // setServoPos(int servo, int target1k)
                    switch (arg1) {
                        case PINCHSERVO:
                            if (arg2 == 1) {
                                claw.clawClosed = true;
                            }
                            if (arg2 == 0) {
                                claw.clawClosed = false;
                            }
                            break;
                        case WRISTSERVO:
                            wPos = arg2;
                            break;
                        case ROTATESERVO:
                            rPos = arg2;
                            break;
                    }
                    CHANGE_LINE = true;
                    break;
                case 3:
//                    followTraj(trajNo)
                    switch (arg1) {
                        case 1:
                            status = 1;
                            Actions.runBlocking(
                                    new SequentialAction(
                                            tab1.build()
                                    )
                            );

                            status = 2;
                            break;

                        case 2:
                            break;
                    }
                    CHANGE_LINE = true;
                    break;

                case 5:
                    // waitTime(int ms)
                    if (prevLine != line) {
                        runtime.reset();
                    }
                    if (runtime.milliseconds() > arg1) {
                        CHANGE_LINE = true;
                    }
                    break;

                case 20:
                    arm.slidesTarget = arg1;
                    CHANGE_LINE = true;
                    break;
                case 21:
                    arm.armTarget = arg1;
                    CHANGE_LINE = true;
                    break;
            }

            prevLine = line;
            if (CHANGE_LINE) {
                line += 1;
            }
        }

//        drive.update();

        // TELEMETRY
        telemetry.addData("Line", line);
        telemetry.addData("Status", status);
        telemetry.update();
    }

    // AUTONOMOUS FUNCTIONS:

    // Hardware Functions
    public void setServoPos(int servo, int target1k) {
        PROGRAM.add(new int[] {1, servo, target1k, 0});
    }

    public void setSlidesTarget(int position) {
        PROGRAM.add(new int[] {20, position, 0, 0});
    }
    public void setPivotTarget(int position) {
        PROGRAM.add(new int[] {21, position, 0, 0});
    }

    // Trajectory Functions
    public void followTraj(int trajNo) {
        PROGRAM.add(new int[] {3, trajNo, 0, 0});
    }

    // Wait Functions
    public void waitTime(int ms) {
        PROGRAM.add(new int[] {5, ms, 0, 0});
    }

    public void buildProgram() {
//        followTraj(1);

//        waitTime(5000);
        setPivotTarget(100);
        waitTime(200);
        setSlidesTarget(100);





    }
}