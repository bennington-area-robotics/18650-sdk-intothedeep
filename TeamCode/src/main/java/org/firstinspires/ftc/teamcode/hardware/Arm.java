package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.FloatRange;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.controllers.PID;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.function.Consumer;

@Config
public class Arm {
    //<editor-fold desc="Config">
    public static float ARM_TICKS_PER_DEGREE = 65f; //this is a good estimate as of 1/24/2025

    public static boolean sacrificialRead = false;
    public static float ARM_TICKS_PER_INCH = 190f;
    public static double ENCODER_TICKS_PER_INCH = 38952.0/37.984;
    public static double MAX_ARM_EXTENSION = 38.5;

    public static double MAX_HORIZONTAL_EXTENSION = 38.0;


    public static double angleTolerance = 0.4;
    public static double DELIVERY_EXTENSION = 38.0;
    public static double DELIVERY_ANGLE = 95.0;

    public static double COLLECTION_EXTENSION = 0.0;
    public static double COLLECTION_ANGLE = -1.0;
    public static double SPECIMEN_ANGLE = 55;

    public static double downwardKP = 0.015, downwardKI = 0, downwardKD = 0.1, downwardKF = 0, downwardMaxI = 0;
    public static double upwardKP = 0.06, upwardKI = 0.001, upwardKD = 0.02, upwardKF = 0.23, upwardMaxI = 0;
    public static double extensionKP = 0.25, extensionKI, extensionKD = 0.2, extensionKF = 0, extensionMaxI;
    public static double retractionKP = 3, retractionKI, retractionKD, retractionKF = 0, retractionMaxI;
    public static double rotationKF = 0.18, rotationKCOS = 1;
    public static double downwardKFMultiplier = 0;
    public static double minThreshold = 0.15;
    public static double verticalKD = 0.1, verticalKP = 0.045;
    public static double KPFactor = 1;

    private final PID downwardPID = new PID(downwardKP, downwardKI, downwardKD, downwardKF, downwardMaxI, 0.75);
    private final PID upwardPID = new PID(upwardKP, upwardKI, upwardKD, upwardKF, upwardMaxI, 0.75);
    private final PID extensionPID = new PID(extensionKP, extensionKI, extensionKD, extensionKF, extensionMaxI, 0.5);
    private final PID retractionPID = new PID(retractionKP, retractionKI, retractionKD, retractionKF, retractionMaxI, 0.5);
    //</editor-fold>

    //<editor-fold desc="Fields">
    private final DcMotorEx angleMotorRight;
    private final DcMotorEx angleMotorLeft;
    private final DcMotorEx extensionMotor;

    private final Encoder angleEncoder;
    private final Encoder extensionEncoder;
    public final TouchSensor tiltLimitSensor;
    public final TouchSensor extensionLimitSensor;

    public boolean extensionRunningToPosition = false;

    LinearOpMode opMode;

    /**
     * Target extension of the arm in inches past the minimum extension (not extended at all)
     */
    private double targetExtension = 0;

    /**
     * Target angle of the arm in degrees relative to the base. 0 is horizontal, while 90 is vertical.
     */
    private double targetAngle = 0;

    private double tickOffsetToZero;

    private double extensionMotorTickOffsetToZero;

    private double cachedAngle;

    private boolean atExtensionTarget;

    private boolean atAngleTarget;

    private Consumer<Arm> runningMacro;

    private AngleMode angleMode = AngleMode.MOVE_TO_TARGET;
    private ExtensionMode extensionMode = ExtensionMode.MOVE_TO_TARGET;

    private double anglePower = 0.0;

    private double extensionPower = 0.0;
    //</editor-fold>

    public enum AngleMode {
        MOVE_TO_TARGET, SET_POWER
    }

    public enum ExtensionMode {
        MOVE_TO_TARGET, SET_POWER
    }

    public Arm(HardwareMap hardwareMap, String tiltMotorLeftName, String tiltMotorRightName, String extensionMotorName, String tiltSensorName, String extensionSensorName, LinearOpMode opMode) {
        //<editor-fold desc="Hardware Config">
        this.angleMotorRight = hardwareMap.get(DcMotorEx.class, tiltMotorRightName);
        this.angleMotorLeft = hardwareMap.get(DcMotorEx.class, tiltMotorLeftName);
        this.extensionMotor = hardwareMap.get(DcMotorEx.class, extensionMotorName);
        this.tiltLimitSensor = hardwareMap.get(TouchSensor.class, tiltSensorName);
        this.extensionLimitSensor = hardwareMap.get(TouchSensor.class, extensionSensorName);
        this.angleEncoder = new Encoder(angleMotorRight);
        this.extensionEncoder = new Encoder(angleMotorLeft);

        this.extensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.angleMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.angleMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        this.angleMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.angleMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.angleMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.angleMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.angleEncoder.setDirection(Encoder.Direction.FORWARD);
        this.extensionEncoder.setDirection(Encoder.Direction.FORWARD);
        this.extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //</editor-fold>


        resetExtension();
        resetAngle();
        resetExtensionEncoder();

        targetAngle = getAngle();
        targetExtension = getExtension();
    }

    /**
     * Get the current angle of the arm. This is relative to the base, at 0 the arm is horizontal, and at 90 the arm is vertical.
     * Uses should be able to handle angles past 90 degrees, since the motor will not always land at exactly 90.
     *
     * @return the angle of the arm relative to the base.
     */
    public double getAngle() {
        return (angleEncoder.getCurrentPosition() - tickOffsetToZero) / ARM_TICKS_PER_DEGREE;
    }

    public double getExtensionEncoderPosition(){
        return (extensionEncoder.getCurrentPosition() - extensionMotorTickOffsetToZero) / ENCODER_TICKS_PER_INCH;
    }

    public double getAngleVelocity(){
        return angleEncoder.getCorrectedVelocity() / ARM_TICKS_PER_DEGREE;
    }

    public boolean isRunningMacro(){return runningMacro!=null;}

    /**
     * Get the current extension of the end of the arm past the minimum extension (fully retracted).
     *
     * @return the extension of the end of the arm.
     */
    public double getExtension(){
        return extensionMotor.getCurrentPosition() / ARM_TICKS_PER_INCH;
    }


    public double getExtensionVelocity(){
        return extensionMotor.getCurrentPosition() / ARM_TICKS_PER_INCH;
    }

    public double getTargetExtension(){
        return targetExtension;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public AngleMode getAngleMode(){
        return angleMode;
    }

    public void setAngleMode(AngleMode angleMode){
        this.angleMode = angleMode;
    }

    public ExtensionMode getExtensionMode(){
        return extensionMode;
    }

    public void setExtensionMode(ExtensionMode extensionMode) {
        this.extensionMode = extensionMode;
    }

    public void killMacro(){
        this.runningMacro = null;
    }

    public void setAnglePower(double anglePower){
        setAngleMode(AngleMode.SET_POWER);
        this.anglePower = anglePower;
    }

    public double getAnglePower() {
        return anglePower;
    }

    public void setExtensionPower(double extensionPower) {
        setExtensionMode(ExtensionMode.SET_POWER);
        this.extensionPower = extensionPower;
    }

    public double getExtensionPower(){
        return extensionPower;
    }

    /**
     * Checks if the passed target angle in inches is valid, then sets the target extension if so.
     * A target angle is valid if the arm will not extend past the horizontal extension limit when at that target angle, and the current target extension.
     *
     * @param degrees the target angle in degrees.
     * @return whether the operation was successful (whether it passed the checks).
     */
    public boolean setTargetAngle(@FloatRange(from=-2, to=100) double degrees){
        setAngleMode(AngleMode.MOVE_TO_TARGET);
        if(runningMacro != null){
            return false;
        }

        if(isValidAngle(degrees)){
            targetAngle = degrees;
            return true;
        }

        return false;
    }

    /**
     * Checks if the passed target angle in inches is valid, then sets the target extension if so.
     * A target angle is valid if the arm will not extend past the horizontal extension limit when at that target angle, and the current target extension.
     *
     * @param degrees the target angle in degrees.
     * @return whether the operation was successful (whether it passed the checks).
     */
    private boolean setTargetAngleIgnoreMacro(@FloatRange(from=0, to=100) double degrees){
        setAngleMode(AngleMode.MOVE_TO_TARGET);
        if(isValidAngle(degrees)){
            targetAngle = degrees;
            return true;
        }

        return false;
    }

    /**
     * Checks if the passed target extension in inches is valid, then sets the target extension if so.
     * A target extension is valid if the arm will not extend past the horizontal extension limit when at the current target angle, and that target extension.
     *
     * @param inches the target extension in inches.
     * @return whether the operation was successful (whether it passed the checks).
     */
    public boolean setTargetExtension(double inches){
        setExtensionMode(ExtensionMode.MOVE_TO_TARGET);
        if(runningMacro != null){
            return false;
        }

        if(isValidExtension(inches)) {
            targetExtension = inches;
            return true;
        }

        return false;
    }

    /**
     * Checks if the passed target extension in inches is valid, then sets the target extension if so.
     * A target extension is valid if the arm will not extend past the horizontal extension limit when at the current target angle, and that target extension.
     *
     * @param inches the target extension in inches.
     * @return whether the operation was successful (whether it passed the checks).
     */
    private boolean setTargetExtensionIgnoreMacro(double inches){
        setExtensionMode(ExtensionMode.MOVE_TO_TARGET);

        if(isValidExtension(inches)) {
            targetExtension = inches;
            return true;
        }

        return false;
    }

    /**
     * Sets the current angle as the zero position.
     */
    public void resetAngle(){
        tickOffsetToZero = angleEncoder.getCurrentPosition();
    }

    public void resetExtensionEncoder() {
        extensionMotorTickOffsetToZero = extensionEncoder.getCurrentPosition();

    }
    public void resetAngleAfterAscent(){
        tickOffsetToZero -= 100 * ARM_TICKS_PER_DEGREE;
    }

    /**
     * Sets the current extension as the zero position.
     */
    public void resetExtension(){
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void deliveryPosition(){
        if(runningMacro != null)
            return;

        setTargetAngleIgnoreMacro(DELIVERY_ANGLE);
        setTargetExtension(DELIVERY_EXTENSION);
    }

    public void specimenPosition(){
        if(runningMacro !=null)
            return;

        setTargetAngleIgnoreMacro(SPECIMEN_ANGLE);
    }

    public void specimenPositionBlocking(){
        if(runningMacro !=null)
            return;

        moveToTargetAngleBlocking(SPECIMEN_ANGLE);
    }

    public double moveToTargetAngleBlocking(double degrees){
        ElapsedTime timer = new ElapsedTime();
        if (!setTargetAngle(degrees))
            return timer.milliseconds();

        while (Math.abs(getAngle() - getTargetAngle()) > 2){
            tick();
        }

        return timer.milliseconds();
    }

    public double moveToTargetExtensionBlocking(double inches){
        ElapsedTime timer = new ElapsedTime();
        if (!setTargetExtension(inches))
            return timer.milliseconds();

        while (Math.abs(getExtensionEncoderPosition() - getTargetExtension()) > 2){
            tick();
        }

        return timer.milliseconds();
    }

    public double moveToTargetAngleBlocking(double degrees, Runnable whileRunning){
        ElapsedTime timer = new ElapsedTime();
        if (!setTargetAngle(degrees))
            return timer.milliseconds();

        while (Math.abs(getAngle() - getTargetAngle()) > 2){
            tick();
            whileRunning.run();
        }

        return timer.milliseconds();
    }

    public double moveToTargetExtensionBlocking(double inches, Runnable whileRunning){
        ElapsedTime timer = new ElapsedTime();
        if (!setTargetExtension(inches))
            return timer.milliseconds();

        while (Math.abs(getExtensionEncoderPosition() - getTargetExtension()) > 2){
            tick();
            whileRunning.run();
        }

        return timer.milliseconds();
    }

    /**
     * Moves arm to collection pose.
     */
    public void collectionPosition(){
        if(getExtensionEncoderPosition() - COLLECTION_EXTENSION < 1.5){
            setTargetExtension(COLLECTION_EXTENSION);
            setTargetAngle(-2);
        }else {
            double inchesPerDegree = (getAngle() - COLLECTION_ANGLE) / (getExtensionEncoderPosition() - COLLECTION_EXTENSION);

            double startAngle = getAngle();
            double startExtension = getExtensionEncoderPosition();

            setTargetExtension(COLLECTION_EXTENSION);
            runningMacro = (arm -> {
                if (Math.abs(COLLECTION_ANGLE - arm.getAngle()) < 10) {
                    arm.setTargetAngleIgnoreMacro(COLLECTION_ANGLE);
                    arm.setTargetExtensionIgnoreMacro(arm.getExtensionEncoderPosition());
                    arm.runningMacro = null;
                } else if (getExtensionEncoderPosition() - COLLECTION_EXTENSION < 1.5) {
                    arm.setTargetAngleIgnoreMacro(COLLECTION_ANGLE);
                    arm.setTargetExtensionIgnoreMacro(DELIVERY_EXTENSION);
                } else {
                    double targetAngle = inchesPerDegree * (arm.getExtensionEncoderPosition() - COLLECTION_EXTENSION) + COLLECTION_ANGLE;
                    arm.setTargetAngleIgnoreMacro(targetAngle);
                }
            });
        }
    }

    int tickCount = 0;

    /**
     * Runs a controller cycle for the arm.
     * This method should be called once per OpMode cycle to maintain the arm's position when at target,
     * or adjust the arm's position when not at target. This controls both extension and retraction.
     */
    public void tick(){

        //sacrifical read to flush the cache
        if(sacrificialRead) {
            angleMotorRight.getCurrentPosition();
        }
        if(tiltLimitSensor.isPressed())
            resetAngle();

        if(extensionLimitSensor.isPressed()) {
            resetExtension();
            resetExtensionEncoder();
        }

        if(runningMacro != null && tickCount % 5 == 0){
            runningMacro.accept(this);
        }
        if(Math.abs(targetAngle - getAngle()) < angleTolerance){
            angleMotorRight.setPower(0);
            angleMotorLeft.setPower(0);
        }
        tickPIDF();
        tickCount++;
    }

    private double lastAnglePower;
    private double calcAnglePower(){
        if(angleMode == AngleMode.MOVE_TO_TARGET) {
            upwardKF = rotationKF * Math.cos(Math.toRadians(getAngle())) * rotationKCOS;
            downwardKF = rotationKF * Math.cos(Math.toRadians(getAngle())) * rotationKCOS * downwardKFMultiplier;
            /*if (getAngle() < 60) {
                downwardKF = rotationKF * Math.cos(Math.toRadians(getAngle())) * rotationKCOS * downwardKFMultiplier;
            }
            else {downwardKF = rotationKF * -1 * Math.cos(Math.toRadians(getAngle()));}*/
            if (targetAngle >= 80) {
                upwardPID.setConstants(verticalKP, upwardKI, verticalKD, 0, upwardMaxI);
            } else {
                upwardPID.setConstants(upwardKP, upwardKI, upwardKD, upwardKF, upwardMaxI);
            }
            downwardPID.setConstants(downwardKP, downwardKI, downwardKD, downwardKF, downwardMaxI);
            double closeKP = upwardKP * KPFactor;
            if (targetAngle - getAngle() < 1.2){
                upwardPID.setConstants(closeKP, upwardKI, upwardKD, upwardKF, upwardMaxI);
            }

            if (targetAngle < getAngle()) {
                lastAnglePower = downwardPID.calc(targetAngle - getAngle());

            }
            else if (targetAngle > getAngle()) {
                lastAnglePower = upwardPID.calc(targetAngle - getAngle());
                if (Math.abs(targetAngle - getAngle()) > 0.1) {  // if we're not at target
                    // Ensure minimum power while maintaining direction
                    lastAnglePower = Math.signum(lastAnglePower) * Math.max(Math.abs(lastAnglePower), minThreshold);
                }
            }
        }else if (angleMode == AngleMode.SET_POWER){
            lastAnglePower = anglePower;
        }
        return lastAnglePower;
    }

    public double getLastAnglePower(){
        return lastAnglePower;
    }

    double lastExtensionPower;
    private double calcExtensionPower(){
        if(extensionMode == ExtensionMode.MOVE_TO_TARGET) {
            extensionPID.setConstants(extensionKP, extensionKI, extensionKD, extensionKF, extensionMaxI);
            retractionPID.setConstants(retractionKP, retractionKI, retractionKD, retractionKF, retractionMaxI);

            if (targetExtension < getExtensionEncoderPosition())
                lastExtensionPower = retractionPID.calc(targetExtension - getExtensionEncoderPosition());
            else if (targetExtension > getExtensionEncoderPosition()) {
                lastExtensionPower = extensionPID.calc(targetExtension - getExtensionEncoderPosition());
            } else {
                lastExtensionPower = 0;
            }
        }else if(extensionMode == ExtensionMode.SET_POWER){
            lastExtensionPower = extensionPower;
        }
        return lastExtensionPower;
    }

    public double getLastExtensionPower(){
        return lastExtensionPower;
    }

    /**
     * Runs a cycle on the PIDF control loop for the arm.
     */
    private void tickPIDF(){
        double anglePower = calcAnglePower();

        angleMotorRight.setPower(anglePower);
        angleMotorLeft.setPower(anglePower);

        extensionMotor.setPower(calcExtensionPower());
    }

    public boolean isValidAngle(double degrees){
        //do not set the target to a degree outside the desired range of motion
        if(degrees < -2.5 || degrees > 100)
            return false;

        //do not set the target to a degree that will cause the arm to move outside the extension bounds.
        //noinspection RedundantIfStatement
        if(targetExtension * Math.cos(Math.toRadians(degrees)) > MAX_HORIZONTAL_EXTENSION)
            return false;

        return true;
    }

    public boolean isValidExtension(double inches){
        if(inches > MAX_ARM_EXTENSION) {
            targetExtension = MAX_ARM_EXTENSION;
            return false;
        }

        if(inches * Math.cos(Math.toRadians(targetAngle)) > MAX_HORIZONTAL_EXTENSION)
            return false;

        targetExtension = inches;
        return true;
    }
}
