package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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
    Pose2d START_POSE = new Pose2d(14, -58, Math.toRadians(90));

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
                .strafeToConstantHeading(new Vector2d(0, -20));

        TrajectoryActionBuilder driveAction2 = drive.actionBuilder(new Pose2d(0, -20, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(0, -36));
//                .strafeToLinearHeading(new Vector2d(20, -33), Math.toRadians(35));

        while (opModeInInit()) {
            arm.initPosition();
            arm.telemetry();
        }

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                new ParallelAction(
                                        (p) -> {arm.armTarget = 450; return false;},
                                        (p) -> {arm.slidesTarget = 150; return false;},
                                        (p) -> {claw.wristTarget = 0.4; return false;},
                                        driveAction1.build()
                                ),
                                (p) -> {claw.wristTarget = 0; return false;},
                                new SleepAction(0.5),
                                (p) -> {claw.clawClosed = false; return false;},
                                driveAction2.build()
                        ),
                        (p) -> {
                            arm.moveAuton();
                            claw.move(arm.slidesPosition);
                            return true;
                        }
                )
        );



    }
}
