package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.controllers.PID;
import org.firstinspires.ftc.teamcode.hardware.controllers.PID.Direction;
import org.firstinspires.ftc.teamcode.util.Encoder;

@Config
public class Collector {

    //config
    public static float WRIST_TICKS_PER_DEGREE = 8192f/360f;
    public static float OPEN_POSITION = 0.4f, CLOSED_POSITION = 0; //grip
    public static int UP_POSITION = 90, DOWN_POSITION = 0; //wrist
    public static float LENGTH = 5f;
    public static double wristKP = 0.012, wristKI, wristKD, wristKF = 0.125, wristMaxI;
    public static double WRIST_OFFSET = 156;

    private final Encoder wristEncoder;

    PID PID = new PID(wristKP, wristKI, wristKD, wristKF, wristMaxI, 1);

    public int wristTarget;

    public final ColorSensor colorSensor;
    final Servo gripServo;
    private final DcMotor wristMotor;
    private WristMode wristMode;
    private final Arm arm;
    private double wristPower = 0.0;

    public enum WristMode {
        MOVE_TO_TARGET, STAY_PARALLEL, STAY_PERPENDICULAR, FLOAT, SET_POWER
    }

    public Collector(Arm arm, HardwareMap hardwareMap, String colorSensorName, String wristMotorName, String gripServoName){
        this.colorSensor = new ColorSensor(hardwareMap, colorSensorName);
        this.gripServo = hardwareMap.get(Servo.class, gripServoName);
        this.wristMotor = hardwareMap.get(DcMotor.class, wristMotorName);
        this.arm = arm;
        this.wristEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, wristMotorName));

        resetPositionAsTop();
        wristMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setWristMode(WristMode.FLOAT);
    }


    //grip


    public void setWristPower(double wristPower) {
        this.wristPower = wristPower;
    }

    public double getWristPower() {
        return wristPower;
    }

    public void openGrip(){
        gripServo.setPosition(OPEN_POSITION);
    }

    public void closeGrip(){
        gripServo.setPosition(CLOSED_POSITION);
    }

    public void setGripPosition(double position) {gripServo.setPosition(position);}

    public boolean isGripOpen(){
        return Helper.errorTolerable(getGripPosition(), OPEN_POSITION, 0.1);
    }

    public boolean isGripClosed(){
        return Helper.errorTolerable(getGripPosition(), CLOSED_POSITION, 0.1);
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
        return Helper.errorTolerable(getWristAngle(), UP_POSITION, 5);
    }

    public boolean isWristDown(){
        return Helper.errorTolerable(getWristAngle(), DOWN_POSITION, 5);
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

    public double getWristAngle(){
        return (wristEncoder.getCurrentPosition() / WRIST_TICKS_PER_DEGREE) + 130;
    }

    public WristMode getWristMode(){
        return wristMode;
    }

    public void setWristMode(WristMode mode){
        wristMode = mode;
        if(wristMode == WristMode.FLOAT){
            wristMotor.setPower(0);
            wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public double getWristVelocity(){
        return wristEncoder.getCorrectedVelocity() / WRIST_TICKS_PER_DEGREE;
    }

    public int getWristTarget() {
        return wristTarget;
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

    public void resetPositionAsTop(){
        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetPositionAs(){
        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void tick(){
        PID.setConstants(wristKP, wristKI, wristKD, wristKF, wristMaxI);

        PID.setDirection(Direction.REVERSE);

        switch (wristMode) {
            case FLOAT:
                wristMotor.setPower(0);
                break;
            case SET_POWER:
                wristMotor.setPower(wristPower);
                break;
            case STAY_PARALLEL:
                wristMotor.setPower(PID.calc(getWristAngle() - (90 - arm.getAngle())));
                break;
            case STAY_PERPENDICULAR:
                wristMotor.setPower(PID.calc(getWristAngle() - (arm.getAngle() - 90)));
                break;
            case MOVE_TO_TARGET:
                wristMotor.setPower(PID.calc(getWristAngle() - wristTarget));
                break;
        }
    }

    private static class Helper {
        private static boolean errorTolerable(Number number1, Number number2, Number tolerance){
            return Math.abs(number2.doubleValue() - number1.doubleValue()) <= tolerance.doubleValue();
        }
    }
}