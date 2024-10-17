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
            throw new IllegalStateException("Color sensor is not an instance of DistanceSensor, this color sensor likely doesn't have distance sensing.");
        }
    }

    public void setGain(float gain){
        colorSensor.setGain(gain);
    }

    public float getGain(){
        return colorSensor.getGain();
    }
}