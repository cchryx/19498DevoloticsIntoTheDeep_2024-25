package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

public class Values {

    public static double TICKS_PER_DEG_GOBUILDA = 537.7 / 360;
    public static double DT_SLOW_FACTOR = 0.3   ;


    // SLIDES VALUES
    public static int SLIDES_MAX_EXTEND = 790;
    public static int SLIDES_HOME = 0;
    public static int SLIDES_SUB = 350;
    public static int SLIDES_HRUNG = 400;
    public static int SLIDES_HBASKET = 740;
    public static int SLIDES_INCR = 5;

    // ARM VALUES
    public static int ARM_MAX = 650;
    public static int ARM_MIN = 0;
    public static int ARM_INCR = 2;
    public static int ARM_HBASKET = 650;
    public static int ARM_HRUNG = 400;
    public static int ARM_WALL = 250;

    // CLAW VALUES
    public static double WRIST_HOME = 0.03;
    public static double WRIST_MIN = 0;
    public static double WRIST_MAX = 0.96;
    public static double WRIST_OFFSET = 0.03;
    public static double WRIST_INCR = 0.01;
    public static double WRIST_HRUNG = 0.5;
    public static double WRIST_WALL = 0.35;


    public static double ROTATE_INIT = 0.545; // 0 deg
    public static double ROTATE_L_MAX = 0; // 90 deg
    public static double ROTATE_R_MAX = 1;
    public static double ROTATE_INCR = 0.0125;
    public static double PINCH_MIN = 0; // Claw close
    public static double PINCH_MAX = 0.3; // Claw open
}
