package org.firstinspires.ftc.teamcode.hardware;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.math.BigDecimal;
import java.math.RoundingMode;

@Config
public class Collector {

    //todo add collector mode to always stay parallel with ground

    //config
    public static float OPEN_POSITION = 0.4f, CLOSED_POSITION = 0; //grip
    public static int UP_POSITION = 90, DOWN_POSITION = 15; //wrist
    public static float LENGTH = 5f;
    public static double wristKP = 0.015, wristKI, wristKD, wristKF = 0.125, wristMaxI;

    PID pid = new PID(wristKP, wristKI, wristKD, wristKF, wristMaxI);

    public int wristTarget;

    public final ColorSensor colorSensor;
    final Servo gripServo;
    private final DcMotor wristMotor;

    @SuppressLint("DefaultLocale")
    public Collector(HardwareMap hardwareMap, String colorSensorName, String wristMotorName, String gripServoName){
        this.colorSensor = new ColorSensor(hardwareMap, colorSensorName);
        this.gripServo = hardwareMap.get(Servo.class, gripServoName);
        this.wristMotor = hardwareMap.get(DcMotor.class, wristMotorName);

        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wristUp();
    }


    //grip

    public void openGrip(){
        gripServo.setPosition(OPEN_POSITION);
    }

    public void closeGrip(){
        gripServo.setPosition(CLOSED_POSITION);
    }

    public boolean isGripOpen(){
        return Helper.round(gripServo.getPosition(), 1) == Helper.round(OPEN_POSITION, 1);
    }

    public boolean isGripClosed(){
        return Helper.round(gripServo.getPosition(), 1) == Helper.round(CLOSED_POSITION, 1);
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

    public void wristTo(int position){
        wristTarget = position;
    }

    public void wristUp(){
        wristTo(UP_POSITION);
    }

    public void wristDown(){
        wristTo(DOWN_POSITION);
    }

    public boolean isWristUp(){
        return Helper.errorTolerable(wristMotor.getCurrentPosition(), UP_POSITION, 5);
    }

    public boolean isWristDown(){
        return Helper.errorTolerable(wristMotor.getCurrentPosition(), DOWN_POSITION, 5);
    }

    public boolean isWristTargetUp(){
        return Helper.errorTolerable(wristTarget, UP_POSITION, 5);
    }

    public boolean isWristTargetDown(){
        return Helper.errorTolerable(wristTarget, DOWN_POSITION, 5);
    }

    public boolean holdingSample(){
        return isGripClosed() && colorSensor.getScoringElementColor() != ScoringElementColor.NONE;
    }

    public boolean holdingSample(ScoringElementColor elementColor){
        return isGripClosed() && colorSensor.getScoringElementColor() == elementColor;
    }

    public double getGripPosition(){
        return gripServo.getPosition();
    }

    public double getWristPosition(){
        return wristMotor.getCurrentPosition();
    }

    /**
     * Attempts to toggle the wrist target between up and down. If the target is not up or down, the target will not be changed.
     * @return whether the wrist target was changed.
     */
    public boolean toggleWrist(){
        if (isWristTargetUp()) {
            wristDown();
            return true;
        }else if (isWristTargetDown()) {
            wristUp();
            return true;
        }else{
            return false;
        }
    }

    public void tick(){
        pid.setConstants(wristKP, wristKI, wristKD, wristKF, wristMaxI);

        pid.setDirection(PID.Direction.REVERSE);
        wristMotor.setPower(pid.tick(wristMotor.getCurrentPosition() - wristTarget));
    }

    private static class Helper {
        private static boolean errorTolerable(int number1, int number2, int tolerance){
            return Math.abs(number2 - number1) <= tolerance;
        }

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