package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpModeCore;
import org.firstinspires.ftc.teamcode.hardware.controllers.PID;
import org.firstinspires.ftc.teamcode.hardware.controllers.PID.Direction;
import org.firstinspires.ftc.teamcode.util.Encoder;

@Config
public class Collector {

    //config
    public static float WRIST_TICKS_PER_DEGREE = 8192f/360f;
    public static float OPEN_POSITION = 0.5f, CLOSED_POSITION = 0.7f; //grip
    public static int UP_POSITION = 90, DOWN_POSITION = 0; //wristMotor
    public static float DEFAULT_POSITION = 1.0f, ROTATED_POSITION = 0.0f;//wristServo
    public static float HALFWAY_POSITION = 0.5f;
    public static float LENGTH = 5f;
    //old pid values for else return 0
    /*public static double upWristKP = 0.008, upWristKI = 0, upWristKD = 0, upWristKF = -0.02, upWristMaxI,  upWristKCOS = -0.15;
    public static double wristOffset = 0;

    public static double downWristKP = 0.0005, downWristKI = 0, downWristKD = 0.003, downWristKF, downWristMaxI, downWristKCOS= 0;
    private boolean withGravity = false;*/

    public static double upWristKP = 0.008, upWristKI = 0, upWristKD = 0, upWristKF = 0, upWristMaxI,  upWristKCOS = -0.13;
    public static double wristOffset = 0;

    public static double downWristKP = 0.0003, downWristKI = 0, downWristKD = 0.001, downWristKF, downWristMaxI, downWristKCOS= 0;
    private boolean withGravity = false;

    double KF = upWristKF;
    private final Encoder wristEncoder;

    PID upwardPID = new PID(upWristKP, upWristKI, upWristKD, upWristKF, upWristMaxI, 1);
    PID downwardPID = new PID(downWristKP, downWristKI, downWristKD, downWristKF, downWristMaxI, 1);
    public double wristTarget;

    //public final ColorSensor colorSensor;
    final Servo gripServo;
    final Servo wristServo;
    private final DcMotor wristMotor;
    private WristMode wristMode;
    private final Arm arm;
    private double wristPower = 0.0;
    public static int submersibleCollectionPosition = 60;

    public static double upPower, downPower = 0;

    public enum WristMode {
        MOVE_TO_TARGET, STAY_PARALLEL, STAY_PERPENDICULAR, FLOAT, SET_POWER
    }

    public Collector(Arm arm, HardwareMap hardwareMap, String colorSensorName, String wristMotorName, String gripServoName, String wristServoName){
        //this.colorSensor = new ColorSensor(hardwareMap, colorSensorName);
        this.gripServo = hardwareMap.get(Servo.class, gripServoName);
        this.wristMotor = hardwareMap.get(DcMotor.class, wristMotorName);
        this.arm = arm;
        this.wristEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, wristMotorName));
        this.wristServo = hardwareMap.get(Servo.class, wristServoName);

        resetPositionAs(0);
        wristMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setWristMode(WristMode.FLOAT);
    }

    public Collector(Arm arm, HardwareMap hardwareMap, String wristMotorName, String gripServoName, String wristServoName){

        this.gripServo = hardwareMap.get(Servo.class, gripServoName);
        this.wristMotor = hardwareMap.get(DcMotor.class, wristMotorName);
        this.arm = arm;
        this.wristEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, wristMotorName));
        this.wristServo = hardwareMap.get(Servo.class, wristServoName);

        resetPositionAs(0);
        wristMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setWristMode(WristMode.FLOAT);
    }

    public double moveWristToBlocking(double angle, Runnable runnable, boolean timerOverride){
        ElapsedTime timer = new ElapsedTime();
        setWristMode(WristMode.MOVE_TO_TARGET);
        wristTo(angle);
        while(Math.abs(getWristAngle() - getWristTarget()) > 2){
            tick();
            runnable.run();
            if (timer.seconds() > 0.5 && !timerOverride){
                return timer.milliseconds();
            }
            if(timer.seconds() > 2 && timerOverride){
                return timer.milliseconds();
            }
        }
        return timer.milliseconds();
    }


    //grip


    public void setWristPower(double wristPower) {
        this.wristPower = wristPower;
    }

    public double getWristPower() {
        return wristMotor.getPower();
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

    public boolean isWithGravity(){return withGravity;}

    public void wristToRotatedPosition(){wristServo.setPosition(ROTATED_POSITION);}
    public void wristToDefaultPosition(){wristServo.setPosition(DEFAULT_POSITION);}
    public void wristToHalfway(){wristServo.setPosition(HALFWAY_POSITION);}

    public void setWristPosition(double position) {wristServo.setPosition(position);}

    public boolean isWristRotated(){
        return Helper.errorTolerable(getWristServoPosition(), ROTATED_POSITION, 0.1);
    }
    public double getWristServoPosition(){
        return wristServo.getPosition();
    }

    public boolean isWristDefault(){
        return Helper.errorTolerable(getWristServoPosition(), DEFAULT_POSITION, 0.1);
    }

    public boolean toggleWristServo(){

        if (isWristDefault()) {
            wristToRotatedPosition();
            while(!isWristRotated()){
                OpModeCore.getInstance().tick();
            }
            setWristMode(WristMode.MOVE_TO_TARGET);
            wristTo(submersibleCollectionPosition);
            return true;
        }else if (isWristRotated()) {
            wristToDefaultPosition();
            setWristMode(WristMode.MOVE_TO_TARGET);
            wristTo(UP_POSITION);
            return true;
        }else{
            return false;
        }
    }

    //wrist

    public void wristTo(double position){
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

    /*public boolean holdingSample(){
        return isGripClosed() && colorSensor.getScoringElementColor() != ScoringElementColor.NONE;
    }

    public boolean holdingSample(ScoringElementColor elementColor){
        return isGripClosed() && colorSensor.getScoringElementColor() == elementColor;
    }*/

    public double getGripPosition(){
        return gripServo.getPosition();
    }

    public double getWristAngle(){
        return (wristEncoder.getCurrentPosition() / WRIST_TICKS_PER_DEGREE) + wristOffset;
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

    public double getWristTarget() {
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
        wristOffset = 130;
    }

    public void resetPositionAs(double angle){
        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wristOffset = angle;
    }

    public void tick(){

        double passedUpKF = upWristKF + ( Math.sin(Math.toRadians(getWristAngle() + arm.getAngle())) * upWristKCOS);
        double passedDownKF = downWristKF + (Math.sin(Math.toRadians(getWristAngle() + arm.getAngle())) * downWristKCOS);


        upwardPID.setConstants(upWristKP, upWristKI, upWristKD, passedUpKF, upWristMaxI);
        downwardPID.setConstants(downWristKP, downWristKI, downWristKD, passedDownKF, downWristMaxI);

        upwardPID.setDirection(Direction.REVERSE);
        downwardPID.setDirection(Direction.REVERSE);

        upPower = upwardPID.calc(getWristAngle() - wristTarget);
        downPower = downwardPID.calc(getWristAngle() - wristTarget);

        switch (wristMode) {
            case FLOAT:
                wristMotor.setPower(0);
                break;
            case SET_POWER:
                wristMotor.setPower(wristPower);
                break;
            case STAY_PARALLEL:
                wristMotor.setPower(upwardPID.calc(getWristAngle() - (90 - arm.getAngle())));
                break;
            case STAY_PERPENDICULAR:
                wristMotor.setPower(upwardPID.calc(getWristAngle() - (arm.getAngle() - 90)));
                break;
            case MOVE_TO_TARGET:
                double gravityFactor = Math.sin(Math.toRadians(getWristAngle() + arm.getAngle()));

                // Calculate the direction we want to move
                double moveDirection = wristTarget - getWristAngle();

                // Determine if we're moving against gravity or with gravity
                if (gravityFactor * moveDirection > 0) {
                    // Moving against gravity - use upward PID controller
                    wristMotor.setPower(upPower);
                    withGravity = false;
                } else {
                    // Moving with gravity - use downward PID controller
                    wristMotor.setPower(downPower);
                    withGravity = true;
                }
                //wristMotor.setPower(upwardPID.calc(getWristAngle() - wristTarget));

                break;
        }
    }

    private static class Helper {
        private static boolean errorTolerable(Number number1, Number number2, Number tolerance){
            return Math.abs(number2.doubleValue() - number1.doubleValue()) <= tolerance.doubleValue();
        }
    }
}