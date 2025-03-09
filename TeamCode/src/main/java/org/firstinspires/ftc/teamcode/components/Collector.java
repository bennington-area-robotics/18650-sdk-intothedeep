package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.SmartEncoder;
import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.SmartServo;
import org.firstinspires.ftc.teamcode.hardware.controllers.GravityPID;
import org.firstinspires.ftc.teamcode.hardware.Direction;

@Config
public class Collector {
    //todo similar to arm, split this into wrist, tilt wrist, and grip

    //config
    public static float WRIST_TICKS_PER_DEGREE = 8192f/360f;
    public static float OPEN_POSITION = 0.4f, CLOSED_POSITION = 0; //grip
    public static int UP_POSITION = 90, DOWN_POSITION = -20; //wrist
    public static float LENGTH = 5f;
    public static double wristKP = 0.009, wristKI, wristKD = 0.02, wristKF = -0.035, wristKG = 0;

    double KF = wristKF;
    private final SmartEncoder wristEncoder;

    GravityPID PID;

    public int wristTarget;

    final SmartServo gripServo;
    private final SmartMotor wristMotor;
    private WristMode wristMode;
    private final TiltBase tiltBase;
    private double wristPower = 0.0;

    public enum WristMode {
        MOVE_TO_TARGET, STAY_PARALLEL, STAY_PERPENDICULAR, FLOAT, SET_POWER
    }

    public Collector(TiltBase tiltBase, String wristMotorName, String gripServoName){
        this.gripServo = Hardware.getServo(gripServoName);
        this.wristMotor = Hardware.getMotor(wristMotorName, true);
        this.tiltBase = tiltBase;
        this.wristEncoder = wristMotor.getEncoder();

        PID = new GravityPID.Builder()
                .forwardKP(() -> wristKP)
                .forwardKI(() -> wristKI)
                .forwardKD(() -> wristKD)
                .forwardKF(() -> wristKF)

                .reverseKP(() -> wristKP)
                .reverseKI(() -> wristKI)
                .reverseKD(() -> wristKD)
                .reverseKF(() -> wristKF)
                .g(() -> wristKG)
                .setGravityFunction((target, actual) -> Math.sin(Math.toRadians(tiltBase.getAngle() + getWristAngle())))
                .tolerance(1)
                .build();

        resetPositionAsTop();
        wristMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setWristMode(WristMode.FLOAT);

    }

    public double moveWristToBlocking(int angle, Runnable runnable){
        ElapsedTime timer = new ElapsedTime();
        setWristMode(WristMode.MOVE_TO_TARGET);
        wristTo(angle);
        while(Math.abs(getWristAngle() - getWristTarget()) > 2){
            tick();
            runnable.run();
            if (timer.seconds() > 0.5){
                return timer.milliseconds();
            }
        }
        return timer.milliseconds();
    }

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

    public boolean isHoldingSample(){
        return isGripClosed();
    }

    public double getGripPosition(){
        return gripServo.getPosition();
    }

    public double getWristAngle(){
        return (wristEncoder.getPosition() / WRIST_TICKS_PER_DEGREE);
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
        return wristEncoder.getVelocity() / WRIST_TICKS_PER_DEGREE;
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
        wristEncoder.resetAs(130);
    }

    public void tick(){
        PID.setDirection(Direction.REVERSE);

        switch (wristMode) {
            case FLOAT:
                wristMotor.setPower(0);
                break;
            case SET_POWER:
                wristMotor.setPower(wristPower);
                break;
            case STAY_PARALLEL:
                wristMotor.setPower(PID.calc(90 - tiltBase.getAngle(), getWristAngle()));
                break;
            case STAY_PERPENDICULAR:
                wristMotor.setPower(PID.calc(tiltBase.getAngle() - 90, getWristAngle()));
                break;
            case MOVE_TO_TARGET:
                wristMotor.setPower(PID.calc(wristTarget, getWristAngle()));
                break;
        }
    }

    private static class Helper {
        private static boolean errorTolerable(Number number1, Number number2, Number tolerance){
            return Math.abs(number2.doubleValue() - number1.doubleValue()) <= tolerance.doubleValue();
        }
    }
}