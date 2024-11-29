package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.components.ArmControl;
import org.firstinspires.ftc.teamcode.components.ClawControl;
import org.firstinspires.ftc.teamcode.components.HardwareInitializer;

@Autonomous(name = "Chamber", group = "Auton")
public class Chamber extends LinearOpMode {
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

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = FtcDashboard.getInstance().getTelemetry();

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


        // Build trajectory actions
        drive = new MecanumDrive(hardwareMap, START_POSE);

        TrajectoryActionBuilder driveAction1 = drive.actionBuilder(START_POSE)
                .strafeToConstantHeading(new Vector2d(0, -18));


//        TrajectoryActionBuilder driveAction2 = driveAction1.fresh().turnTo(Math.toRadians(-28))
//                .strafeToLinearHeading(new Vector2d(36, -42), Math.toRadians(35));

        while (opModeInInit()) {}

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                new ParallelAction(
                                        driveAction1.build(),
                                        (p) -> {arm.armTarget = 450; return false;},
                                        (p) -> {arm.slidesTarget = 150; return false;},
                                        (p) -> {claw.wristTarget = 0.4; return false;}
                                ),
                                (p) -> {claw.wristTarget = 0.2; return false;},
                                (p) -> {arm.armTarget = 400; return false;}
                        ),
                        (p) -> {
                            arm.move("auton");
                            claw.move(arm.slidesPosition, "auton");
                            return true;
                        }
                )
        );



    }
}
