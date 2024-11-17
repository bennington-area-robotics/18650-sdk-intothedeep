package org.firstinspires.ftc.teamcode.hardware;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.jetbrains.annotations.NotNull;

public class ColorSensor {
    NormalizedColorSensor colorSensor;

    public ColorSensor(HardwareMap hardwareMap, String name) {
        this.colorSensor = hardwareMap.get(NormalizedColorSensor.class, name);
    }

    /**
     * Reads the current color and returns it as RGB values.
     * @return NormalizedRGBA with the color detected.
     */
    public NormalizedRGBA getRGBA(){
        return colorSensor.getNormalizedColors();
    }

    /**
     * Reads the current color and returns it as HSV values.
     * @return array containing the Hue, Saturation and Value floats in that order.
     */
    public float[] getHSV() {
        float[] hsv = new float[3];
        Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);
        if(hsv[0] == 0 && hsv[1] == 0 && hsv[2] == 0)
            throw new RuntimeException("HSV values failed to read from ColorSensor");
        return hsv;
    }

    /**
     * @param distanceUnit The unit the distance should be returned in
     * @return distance to the closest obstruction directly in front of the color sensor
     */
    public double getDistance(DistanceUnit distanceUnit){
        if (colorSensor instanceof DistanceSensor) {
            return ((DistanceSensor) colorSensor).getDistance(distanceUnit);
        }else {
            throw new IllegalStateException("Color sensor is not an instance of DistanceSensor, this color sensor likely doesn't support distance sensing.");
        }
    }

    // Threshold for hue matching
    private static final float THRESHOLD = 20.0f;

    /**
     * Reads the currently detected color and returns a scoring element color or null if no scoring element color was detected.
     *
     * @return the approximate color detected by the sensor. If no scoring element color is detected returns ScoringElementColor.NONE.
     */
    public ScoringElementColor getScoringElementColor() {
        float[] hsv = getHSV();

        float hue = hsv[0];
        final float saturation = hsv[1];
        final float value = hsv[2];

        // Target hues for colors
        final float RED_HUE = 0.0f;
        final float YELLOW_HUE = 60.0f;
        final float BLUE_HUE = 240.0f;

        // Ensure valid saturation and value
        if (saturation < 0.2 || value < 0.4) {
            return ScoringElementColor.NONE; // Very low saturation or brightness, return None
        }

        // Normalize hue
        hue = hue % 360;
        if (hue < 0) hue += 360;

        // Check closeness to each color
        if (isWithinThreshold(hue, RED_HUE) || isWithinThreshold(value, 360.0f)) {
            return ScoringElementColor.RED;
        } else if (isWithinThreshold(hue, YELLOW_HUE)) {
            return ScoringElementColor.YELLOW;
        } else if (isWithinThreshold(hue, BLUE_HUE)) {
            return ScoringElementColor.BLUE;
        } else {
            return ScoringElementColor.NONE;
        }
    }

    private static boolean isWithinThreshold(float hue, float targetHue) {
        return Math.abs(hue - targetHue) <= THRESHOLD;
    }


    public void setGain(float gain){
        colorSensor.setGain(gain);
    }

    public float getGain(){
        return colorSensor.getGain();
    }
}