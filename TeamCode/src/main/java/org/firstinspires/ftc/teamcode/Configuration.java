package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Configuration {
    // Note: f after a number denotes a float, meaning it is not a double as putting just a .0 will denote,
    // but will be treated as a decimal and therefor division with remainder will be calculated.

    //Collector
    public static final float OPEN_POSITION = 1, CLOSED_POSITION = 0; //grip
    public static final float UP_POSITION = 0, DOWN_POSITION = 0.46f; //wrist
    public static float COLLECTOR_LENGTH = 5f;

    //Color sensor
    public static float HUE_THRESHOLD = 20.0f;
    public static float RED_HUE = 25.0f;
    public static float YELLOW_HUE = 75.0f;
    public static float BLUE_HUE = 215.0f;

    public static float MIN_VALUE = 0.2f;
    public static float MIN_SATURATION = 0.2f;

    public static int GAIN = 150;

    //Arm

    private static final float ARM_GEAR_RATIO = 86.0f / 28.0f;
    private static final float ARM_TICKS_PER_DEGREE_AT_MOTOR_OUTPUT = 288.0f/360.0f;

    public static float ARM_TICKS_PER_DEGREE = ARM_GEAR_RATIO * ARM_TICKS_PER_DEGREE_AT_MOTOR_OUTPUT;

    public static float ARM_TICKS_PER_INCH = 411.6f; //TODO change this to a real number
    public static float MAX_ARM_EXTENSION = 38f;

    public static float MAX_HORIZONTAL_EXTENSION = 38f;

    public static float LOW_POWER_MODIFIER = 0.25f;
    public static float HIGH_POWER_MODIFIER = 0.75f;
}
