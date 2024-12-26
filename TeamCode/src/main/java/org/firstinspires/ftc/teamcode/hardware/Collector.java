package org.firstinspires.ftc.teamcode.hardware;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.OpModeCore;

import java.math.BigDecimal;
import java.math.RoundingMode;

@Config
public class Collector {
    //config
    public static final float OPEN_POSITION = 1, CLOSED_POSITION = 0; //grip
    public static final float UP_POSITION = 0, DOWN_POSITION = 0.46f; //wrist
    public static float LENGTH = 5f;

    public final ColorSensor colorSensor;
    final Servo gripServo, wristServo;

    @SuppressLint("DefaultLocale")
    public Collector(HardwareMap hardwareMap, String colorSensorName, String wristServoName, String gripServoName, boolean includeTelem){
        this.colorSensor = new ColorSensor(hardwareMap, colorSensorName);
        this.gripServo = hardwareMap.get(Servo.class, gripServoName);
        this.wristServo = hardwareMap.get(Servo.class, wristServoName);

        if(includeTelem) {
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
                        float[] hsv = colorSensor.getHSV();
                        return String.format("Hue: %.3f Saturation: %.3f Value: %.3f", hsv[0], hsv[1], hsv[2]);
                    })
                    .addData("RGB", () -> {
                        NormalizedRGBA rgba = colorSensor.getRGBA();
                        return String.format("Red: %.3f Green: %.3f Blue: %.3f", rgba.red, rgba.green, rgba.blue);
                    })
                    .addData("Scoring Color", colorSensor::getScoringElementColor);
        }
    }


    //grip

    public void openGrip(){
        gripServo.setPosition(OPEN_POSITION);
    }

    public void closeGrip(){
        gripServo.setPosition(CLOSED_POSITION);
    }

    public boolean isGripOpen(){
        return Helper.round(gripServo.getPosition(), 1) == OPEN_POSITION;
    }

    public boolean isGripClosed(){
        return Helper.round(gripServo.getPosition(), 1) == CLOSED_POSITION;
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

    public void wrist(double position){
        wristServo.setPosition(position);
    }

    public void wristUp(){
        wristServo.setPosition(UP_POSITION);
    }

    public void wristDown(){
        wristServo.setPosition(DOWN_POSITION);
    }

    public boolean isWristUp(){
        return Helper.round(wristServo.getPosition(), 1) == UP_POSITION;
    }

    public boolean isWristDown(){
        return Helper.round(wristServo.getPosition(), 1) == DOWN_POSITION;
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