package org.firstinspires.ftc.teamcode.hardware;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
    public float[] getHSV(){
        float[] hsv = new float[3];
        Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);
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
     * @return the approximate color detected by the sensor. If no scoring element color is detected returns null.
     */
    public ScoringElementColor getColorName() {
        float[] hsv = getHSV();

        final float h = hsv[0];
        final float s = hsv[1];
        float v = hsv[2];



        // Target hues for colors
        final float RED_HUE = 0.0f;
        final float YELLOW_HUE = 60.0f;
        final float BLUE_HUE = 240.0f;

        // Ensure valid saturation and value
        if (s < 0.2 || v < 0.4) {
            return null; // Very low saturation or brightness, return None
        }

        // Normalize hue
        v = v % 360;
        if (v < 0) v += 360;

        // Check closeness to each color
        if (isWithinThreshold(v, RED_HUE) || isWithinThreshold(v, 360.0f)) {
            return ScoringElementColor.RED;
        } else if (isWithinThreshold(v, YELLOW_HUE)) {
            return ScoringElementColor.YELLOW;
        } else if (isWithinThreshold(v, BLUE_HUE)) {
            return ScoringElementColor.BLUE;
        } else {
            return null;
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