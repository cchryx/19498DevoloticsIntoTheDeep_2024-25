package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class LightControl {

    // Method to set the light pattern based on a color string
    public static void setLightPattern(RevBlinkinLedDriver ledDriver, String color) {
        RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.RED; // default pattern

        if (color != null) {
            switch (color.toUpperCase()) {
                case "RED":
                    pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                    break;
                case "GREEN":
                    pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                    break;
                case "BLUE":
                    pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
                    break;
                case "BLUE_GREEN":
                    pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN;
                    break;
                case "VIOLET":
                    pattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
                    break;
                case "YELLOW":
                    pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                    break;
                case "ORANGE":
                    pattern = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
                    break;
                case "WHITE":
                    pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
                    break;
                case "RUNNING":
                    pattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE;
                    break;
                case "OFF":
                    pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
                    break;
            }
        }

        ledDriver.setPattern(pattern);
    }
}
