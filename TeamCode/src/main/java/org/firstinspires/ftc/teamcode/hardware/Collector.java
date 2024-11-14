package org.firstinspires.ftc.teamcode.hardware;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.OpModeCore;

import java.math.BigDecimal;
import java.math.RoundingMode;

public class Collector {
    public final ColorSensor colorSensor;
    final Servo gripServo, wristServo;
    public static final float openPosition = 1, closedPosition = 0;

    public static final float upPosition = 1, downPosition = 0;

    @SuppressLint("DefaultLocale")
    public Collector(HardwareMap hardwareMap, String colorSensorName, String wristServoName, String gripServoName){
        this.colorSensor = new ColorSensor(hardwareMap, colorSensorName);
        this.gripServo = hardwareMap.get(Servo.class, gripServoName);
        this.wristServo = hardwareMap.get(Servo.class, wristServoName);

        OpModeCore.getTelemetry().addLine("Grip")
                .addData("Position", gripServo::getPosition)
                .addData("Open?", this::isGripOpen)
                .addData("Closed?", this::isGripClosed);
        OpModeCore.getTelemetry().addLine("Wrist")
                .addData("Position", wristServo::getPosition)
                .addData("Up?", this::isWristUp)
                .addData("Down?", this::isWristDown);
        OpModeCore.getTelemetry().addLine("Color Sensor")
                .addData("HSV", () -> {
                    float[] hsvValues = colorSensor.getHSV(); // get and store hsv values so we are using the same sample for each value
                    return String.format("Hue: %.3f Saturation: %.3f Value: %.3f", hsvValues[0], hsvValues[1], hsvValues[2]);
                })
                .addData("Scoring Color", colorSensor::getScoringElementColor);

    }


    //grip

    public void openGrip(){
        gripServo.setPosition(openPosition);
    }

    public void closeGrip(){
        gripServo.setPosition(closedPosition);
    }

    public boolean isGripOpen(){
        return Helper.round(gripServo.getPosition(), 1) == openPosition;
    }

    public boolean isGripClosed(){
        return Helper.round(gripServo.getPosition(), 1) == closedPosition;
    }

    /**
     * Checks the current position and if detected it as open or closed, closes it or opens it respectively.
     * If position is not close enough to a closed or open position to estimate, it does nothing and returns false.
     * @return whether the grip was toggled
     */
    public boolean toggleGrip(){
        if (isGripOpen()) {
            closeGrip();
            return true;
        }else if (isGripClosed()) {
            openGrip();
            return true;
        }else{
            return false;
        }
    }

    //wrist

    public void wristUp(){
        wristServo.setPosition(upPosition);
    }

    public void wristDown(){
        wristServo.setPosition(downPosition);
    }

    public boolean isWristUp(){
        return Helper.round(wristServo.getPosition(), 1) == upPosition;
    }

    public boolean isWristDown(){
        return Helper.round(wristServo.getPosition(), 1) == downPosition;
    }

    /**
     * Checks the current position of the wrist and if detected it as up or down, moves it down or moves it up respectively.
     * If position is not close enough to a up or down position to estimate, it does nothing and returns false.
     * @return whether the wrist was toggled
     */
    public boolean toggleWrist(){
        if (isWristUp()) {
            wristDown();
            return true;
        }else if (isWristDown()) {
            wristUp();
            return true;
        }else{
            return false;
        }
    }

    private static class Helper {
        public static double round(double value, int precision) {
            if (precision < 0) {
                throw new IllegalArgumentException("Precision must be a non-negative integer.");
            }
            BigDecimal bd = BigDecimal.valueOf(value);
            bd = bd.setScale(precision, RoundingMode.HALF_UP);
            return bd.doubleValue();
        }
    }
}