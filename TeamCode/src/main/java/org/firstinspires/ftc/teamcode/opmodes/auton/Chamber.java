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
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.components.ArmControl;
import org.firstinspires.ftc.teamcode.components.ClawControl;
import org.firstinspires.ftc.teamcode.components.HardwareInitializer;
import org.firstinspires.ftc.teamcode.components.Values;

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
    Pose2d START_POSE = new Pose2d(14, -56, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = FtcDashboard.getInstance().getTelemetry();

        // Initialize hardware
        hardwareInitializer = new HardwareInitializer();
        hardwareInitializer.initHardware(hardwareMap);

        SLIDES_F = hardwareInitializer.getMotor("SLIDESF");
        SLIDES_B = hardwareInitializer.getMotor("SLIDESB");
        ARM = hardwareInitializer.getMotor("ARM");
        ROTATE = hardwareInitializer.getServoEx("ROTATE");
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

        TrajectoryActionBuilder score1 = drive.actionBuilder(START_POSE)
                .strafeToConstantHeading(new Vector2d(0, -19));

        TrajectoryActionBuilder back1 = drive.actionBuilder(new Pose2d(0, -19, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(20, -32));

        TrajectoryActionBuilder right1 = drive.actionBuilder(new Pose2d(20, -32, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(36, -32));

        TrajectoryActionBuilder up1 = drive.actionBuilder(new Pose2d(36, -32, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(36, -12));

        TrajectoryActionBuilder right1_2 = drive.actionBuilder(new Pose2d(36, -12, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(42, -12));

        TrajectoryActionBuilder back1_2 = drive.actionBuilder(new Pose2d(42, -12, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(42, -44));


        //SECOND SAMPLE
        TrajectoryActionBuilder up2 = drive.actionBuilder(new Pose2d(42, -44, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(42, -15));

        TrajectoryActionBuilder right2 = drive.actionBuilder(new Pose2d(42, -15, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(53, -17));

        TrajectoryActionBuilder back2 = drive.actionBuilder(new Pose2d(53, -17, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(53, -50));


        //PICKUP
        TrajectoryActionBuilder pickup1 = drive.actionBuilder(new Pose2d(53, -50, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(38, -46), Math.toRadians(270));

        TrajectoryActionBuilder pickup2 = drive.actionBuilder(new Pose2d(38, -45, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(38, -49), Math.toRadians(270));

        TrajectoryActionBuilder alignScore = drive.actionBuilder(new Pose2d(38, -49, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(3, -50), Math.toRadians(90));

        TrajectoryActionBuilder score2 = drive.actionBuilder(new Pose2d(3, -50, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(3, -23), Math.toRadians(90));

        TrajectoryActionBuilder alignPark = drive.actionBuilder(new Pose2d(3, -20, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(3, -32), Math.toRadians(90));

        TrajectoryActionBuilder park = drive.actionBuilder(new Pose2d(3, -32, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(38, -50), Math.toRadians(90));




        //OBSOLETE
        //THIRD SAMPLE
//        TrajectoryActionBuilder up3 = drive.actionBuilder(new Pose2d(53, -60, Math.toRadians(90)))
//                .strafeToConstantHeading(new Vector2d(53, -29));
//
//        TrajectoryActionBuilder right3 = drive.actionBuilder(new Pose2d(53, -29, Math.toRadians(90)))
//                .strafeToConstantHeading(new Vector2d(60, -30));
//
//        TrajectoryActionBuilder back3 = drive.actionBuilder(new Pose2d(60, -30, Math.toRadians(90)))
//                .strafeToConstantHeading(new Vector2d(60, -64));


        while (opModeInInit()) {
            arm.initPosition();
            arm.telemetry();
        }

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                //arm positions and then moving to scoring position
                                new ParallelAction(
                                        (p) -> {arm.armTarget = 440; return false;},
                                        (p) -> {arm.slidesTarget = 150; return false;},
                                        (p) -> {claw.wristTarget = 0.38; return false;},
                                        score1.build()
                                ),

                                //moving the wrist down
                                (p) -> {claw.wristTarget = Values.WRIST_HOME; return false;},
                                (p) -> {arm.armTarget = 420; return false;},


                                //waiting 0.5 seconds
                                new SleepAction(0.5),
                                //open the claw
                                (p) -> {claw.clawClosed = false; return false;},
                                new SleepAction(0.5),
//                                (p) -> {claw.wristTarget = 0.4; return false;},
//                                new SleepAction(0.5),


                                // moving the arm back in
                                new ParallelAction(
                                        (p) -> {arm.slidesTarget = Values.SLIDES_HOME; return false;},
                                        (p) -> {claw.wristTarget = Values.WRIST_MAX; return false;},
                                        back1.build()
                                ),
                                new SleepAction(0.5),
                                (p) -> {arm.armTarget = 200; return false;},


                                //PUSHING FIRST SAMPLE
                                new ParallelAction(
                                        (p) -> {arm.armTarget = Values.ARM_HOME + 50; return false;},
                                        (p) -> {arm.slidesTarget = Values.SLIDES_HOME; return false;},
                                        (p) -> {claw.wristTarget = Values.WRIST_MAX; return false;},
                                        right1.build()
                                ),
                                up1.build(),
                                right1_2.build(),
                                back1_2.build(),

                                //PUSHING SECOND SAMPLE
                                up2.build(),
                                right2.build(),
                                back2.build(),

                                //SCORE 2
                                pickup1.build(),
                                new ParallelAction(
                                        (p) -> {arm.armTarget = Values.ARM_WALL; return false;},
                                        (p) -> {arm.slidesTarget = Values.SLIDES_WALL; return false;},
                                        (p) -> {claw.wristTarget = Values.WRIST_WALL; return false;}
                                ),

                                pickup2.build(),

                                (p) -> {claw.clawClosed = true; return false;},
                                new SleepAction(1),

                                (p) -> {claw.wristTarget = Values.WRIST_WALL_UP; return false;},
                                (p) -> {arm.armTarget = Values.ARM_WALL + 100; return false;},


                                new SleepAction(1),

                                new ParallelAction(
                                        (p) -> {arm.armTarget = Values.ARM_HOME; return false;},
                                        (p) -> {arm.slidesTarget = Values.SLIDES_HOME; return false;},
                                        (p) -> {claw.wristTarget = Values.WRIST_MAX; return false;},
                                        alignScore.build()
                                ),

                                new ParallelAction(
                                        (p) -> {arm.armTarget = 440; return false;},
                                        (p) -> {arm.slidesTarget = 150; return false;},
                                        (p) -> {claw.wristTarget = 0.38; return false;},
                                        score2.build()
                                        ),

                                new SleepAction(0.5),
                                //moving the wrist down
                                (p) -> {claw.wristTarget = Values.WRIST_HOME; return false;},
                                (p) -> {arm.armTarget = 420; return false;},

                                //waiting 0.5 seconds
                                new SleepAction(0.5),
                                //open the claw
                                (p) -> {claw.clawClosed = false; return false;},
                                new SleepAction(0.5),
//                                (p) -> {claw.wristTarget = 0.4; return false;},
//                                new SleepAction(0.5),

                                // moving the arm back in
                                new ParallelAction(
                                        (p) -> {arm.slidesTarget = Values.SLIDES_HOME; return false;},
                                        (p) -> {claw.wristTarget = Values.WRIST_MAX; return false;},
                                        alignPark.build()
                                ),
                                new SleepAction(0.5),
                                (p) -> {arm.armTarget = 200; return false;},
                                park.build()


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
