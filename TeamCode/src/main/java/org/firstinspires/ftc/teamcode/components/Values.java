package org.firstinspires.ftc.teamcode.components;

public class Values {

    public static double TICKS_PER_DEG_GOBUILDA = 537.7 / 360;
    public static double DT_SLOW_FACTOR = 0.3   ;


    // SLIDES VALUES
    public static int SLIDES_MAX_EXTEND = 790;
    public static int SLIDES_HOME = 0;
    public static int SLIDES_SUB = 250;
    public static int SLIDES_HRUNG = 140;
    public static int SLIDES_HRUNG_S = 220;
    public static int SLIDES_HBASKET = 740;
    public static int SLIDES_WALL = 50;
    public static int SLIDES_INCR = 5;
    public static int SLIDES_DROPOFF = 300; // Claw open

    // ARM VALUES
    public static int ARM_MAX = 650;
    public static int ARM_MIN = 0;
    public static int ARM_HOME = 100;
    public static int ARM_INCR = 2;
    public static int ARM_SUB = 85;
    public static int ARM_HBASKET = 580;
    public static int ARM_HRUNG = 550;
    public static int ARM_WALL = 120;

    // CLAW VALUES
    public static double WRIST_HOME = 0.03;
    public static double WRIST_MIN = 0;
    public static double WRIST_MED = 0.4;
    public static double WRIST_MAX = 0.96;
    public static double WRIST_SUB = 0.65;
    public static double WRIST_OFFSET = 0.03;
    public static double WRIST_INCR = -0.03;
    public static double WRIST_HRUNG = 0.9;
    public static double WRIST_HRUNG_S = 0.7;
    public static double WRIST_WALL = 0.45;
    public static double WRIST_WALL_UP = 0.65;


    public static double ROTATE_INIT = 0.335; // 0 deg
    public static double ROTATE_L_MAX = 0; // 90 deg
    public static double ROTATE_R_MAX = 1;
    public static double ROTATE_INCR = 0.03;

    public static double PINCH_MIN = 0; // Claw close
    public static double PINCH_MAX = 0.3; // Claw open


}
