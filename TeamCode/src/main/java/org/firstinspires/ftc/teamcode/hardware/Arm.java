package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.FloatRange;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.OpModeCore;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.function.Consumer;

@Config
public class Arm {
    //config
        public static float ARM_TICKS_PER_DEGREE = 65f; //this is a good estimate as of 1/24/2025

        public static float ARM_TICKS_PER_INCH = 190f;
        public static double MAX_ARM_EXTENSION = 38.0;

        public static double MAX_HORIZONTAL_EXTENSION = 38.0;

        public static double DELIVERY_EXTENSION = 38.0;
        public static double DELIVERY_ANGLE = 95.0;

        public static double COLLECTION_EXTENSION = 0.0;
        public static double COLLECTION_ANGLE = 0.0;
        public static double SPECIMEN_ANGLE = 55;
    //config

    private final DcMotorEx angleMotorRight;
    private final DcMotorEx angleMotorLeft;
    private final DcMotorEx extensionMotor;

    private final Encoder angleEncoder;
    private final TouchSensor touchSensor;

    /**
     * Target extension of the arm in inches past the minimum extension (not extended at all)
     */
    private double targetExtension = 0;

    /**
     * Target angle of the arm in degrees relative to the base. 0 is horizontal, while 90 is vertical.
     */
    private double targetAngle = 0;

    private double tickOffsetToZero;

    private double cachedAngle;

    private boolean atExtensionTarget;

    private boolean atAngleTarget;

    private Consumer<Arm> runningMacro;

    //todo these need actual trained values
    public static double downwardKP = 0.005, downwardKI = 0, downwardKD = 0, downwardKF = -0.15, downwardMaxI = 0;
    public static double upwardKP = 0.02, upwardKI = 0.00001, upwardKD = 0.2, upwardKF = 0.15, upwardMaxI = 0.09;
    public static double extensionKP = 0.1, extensionKI, extensionKD, extensionKF = 0.15, extensionMaxI;
    public static double retractionKP = 0.1, retractionKI, retractionKD, retractionKF = 0.2, retractionMaxI;

    private final PID downwardPID = new PID(downwardKP, downwardKI, downwardKD, downwardKF, downwardMaxI).setTolerance(0.75);
    private final PID upwardPID = new PID(upwardKP, upwardKI, upwardKD, upwardKF, upwardMaxI).setTolerance(0.75);
    private final PID extensionPID = new PID(extensionKP, extensionKI, extensionKD, extensionKF, extensionMaxI).setTolerance(0.2);
    private final PID retractionPID = new PID(retractionKP, retractionKI, retractionKD, retractionKF, retractionMaxI).setTolerance(0.2);


    public Arm(HardwareMap hardwareMap, String tiltMotorLeftName, String tiltMotorRightName, String extensionMotorName, String touchSensorName) {
        this.angleMotorRight = hardwareMap.get(DcMotorEx.class, tiltMotorRightName);
        this.angleMotorLeft = hardwareMap.get(DcMotorEx.class, tiltMotorLeftName);
        this.extensionMotor = hardwareMap.get(DcMotorEx.class, extensionMotorName);
        this.touchSensor = hardwareMap.get(TouchSensor.class, touchSensorName);
        this.angleEncoder = new Encoder(angleMotorRight);

        this.extensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.angleMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.angleMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        this.angleMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.angleMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.angleMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.angleMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.angleEncoder.setDirection(Encoder.Direction.FORWARD);

        resetExtension();
        resetAngle();

        targetAngle = getAngle();
        targetExtension = getExtension();

        OpModeCore.getTelemetry().addData("Current Arm Angle", this::getCachedAngle);
        OpModeCore.getTelemetry().addData("Target Arm Angle", this::getTargetAngle);
        OpModeCore.getTelemetry().addData("Current Arm Extension", this::getCachedExtension);
        OpModeCore.getTelemetry().addData("Target Arm Extension", this::getTargetExtension);
        OpModeCore.getTelemetry().addData("Last Angle Power", this::getLastAnglePower);
        OpModeCore.getTelemetry().addData("Last Extension Power", this::getLastExtensionPower);
        OpModeCore.getTelemetry().addData("Touch Pressed", touchSensor::isPressed);
        OpModeCore.getTelemetry().addData("Arm Encoder Ticks", () -> angleEncoder.getCurrentPosition() - tickOffsetToZero);
    }

    /**
     * Get the current angle of the arm. This is relative to the base, at 0 the arm is horizontal, and at 90 the arm is vertical.
     * Uses should be able to handle angles past 90 degrees, since the motor will not always land at exactly 90.
     *
     * @apiNote This method is relatively costly due to reading motor positions, avoid calling more than necessary. Use to update the cached angle when necessary.
     * @return the angle of the arm relative to the base.
     */
    public double getAngle(){
        cachedAngle = ( angleEncoder.getCurrentPosition() - tickOffsetToZero) / ARM_TICKS_PER_DEGREE;
        return cachedAngle;
    }

    /**
     * Returns the cached angle value.
     * <p>
     * This method provides the last angle value retrieved from the encoder.
     * Using the cached value can improve performance by avoiding redundant
     * calls to hardware components.
     * </p>
     *
     * @return The cached angle value.
     */
    public double getCachedAngle(){
        return cachedAngle;
    }

    private double cachedExtension;
    public double getCachedExtension(){
        return cachedExtension;
    }

    /**
     * Get the current extension of the end of the arm past the minimum extension (fully retracted).
     *
     * @apiNote This method is relatively costly due to reading motor positions, avoid calling more than necessary.
     * @return the extension of the end of the arm.
     */
    public double getExtension(){
        cachedExtension = extensionMotor.getCurrentPosition() / ARM_TICKS_PER_INCH;
        return cachedExtension;
    }

    public double getTargetExtension(){
        return targetExtension;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    /**
     * Checks if the passed target angle in inches is valid, then sets the target extension if so.
     * A target angle is valid if the arm will not extend past the horizontal extension limit when at that target angle, and the current target extension.
     *
     * @param degrees the target angle in degrees.
     * @return whether the operation was successful (whether it passed the checks).
     */
    public boolean setTargetAngle(@FloatRange(from=0, to=100) double degrees){
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
     * Sets the current angle as the zero position.
     */
    public void resetAngle(){
        tickOffsetToZero = angleEncoder.getCurrentPosition();
    }

    /**
     * Sets the current extension as the target and zero position.
     */
    public void resetExtension(){
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        targetExtension = 0;
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private double getCollectorExtension(){
        if(OpModeCore.getCollector().isWristUp())
            return Collector.LENGTH;
        return 0;
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

    /**
     * Moves arm to collection pose.
     */
    public void collectionPosition(){
        if(getCachedExtension() - COLLECTION_EXTENSION == 0){
            setTargetExtension(COLLECTION_EXTENSION);
            setTargetAngle(0);
        }else {
            double inchesPerDegree = (getCachedAngle() - COLLECTION_ANGLE) / (getCachedExtension() - COLLECTION_EXTENSION);

            double startAngle = getCachedAngle();
            double startExtension = getCachedExtension();

            setTargetExtension(COLLECTION_EXTENSION);
            runningMacro = (arm -> {
                if (Math.abs(COLLECTION_ANGLE - arm.getCachedAngle()) < 0.5) {
                    arm.runningMacro = null;
                } else {
                    double targetAngle = inchesPerDegree * (arm.getCachedExtension() - COLLECTION_EXTENSION) + COLLECTION_ANGLE;
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
        if(touchSensor.isPressed())
            resetAngle();

        //update the caches
        getAngle();
        getExtension();

        if(runningMacro != null && tickCount % 5 == 0){
            runningMacro.accept(this);
        }

        tickPIDF();
        tickCount++;
    }

    private double lastAnglePower;
    private double getAnglePower(){
        downwardPID.setConstants(downwardKP, downwardKI, downwardKD,downwardKF, downwardMaxI);
        upwardPID.setConstants(upwardKP, upwardKI, upwardKD, upwardKF, upwardMaxI);

        if(targetAngle < getCachedAngle())
            lastAnglePower = downwardPID.tick(targetAngle - getCachedAngle());
        else if (targetAngle > getCachedAngle()){
            lastAnglePower = upwardPID.tick(targetAngle - getCachedAngle());
        }else{
            lastAnglePower = 0;
        }
        return lastAnglePower;
    }

    private double getLastAnglePower(){
        return lastAnglePower;
    }

    double lastExtensionPower;
    private double getExtensionPower(){
        extensionPID.setConstants(extensionKP, extensionKI, extensionKD, extensionKF, extensionMaxI);
        retractionPID.setConstants(retractionKP, retractionKI, retractionKD, retractionKF, retractionMaxI);

        if(targetExtension < getCachedExtension())
            lastExtensionPower = retractionPID.tick(targetExtension - getCachedExtension());
        else if (targetExtension > getCachedExtension()){
            lastExtensionPower = extensionPID.tick(targetExtension - getCachedExtension());
        }else{
            lastExtensionPower = 0;
        }
        return lastExtensionPower;
    }

    private double getLastExtensionPower(){
        return lastExtensionPower;
    }

    /**
     * Runs a cycle on the PIDF control loop for the arm.
     */
    private void tickPIDF(){
        double anglePower = getAnglePower();

        angleMotorRight.setPower(anglePower);
        angleMotorLeft.setPower(anglePower);

        extensionMotor.setPower(getExtensionPower());
    }

    public boolean isValidAngle(double degrees){
        //do not set the target to a degree outside the desired range of motion
        if(degrees < 0 || degrees > 100)
            return false;

        //do not set the target to a degree that will cause the arm to move outside the extension bounds.
        //noinspection RedundantIfStatement
        if(targetExtension * Math.cos(Math.toRadians(degrees)) > MAX_HORIZONTAL_EXTENSION)
            return false;

        return true;
    }

    public boolean isValidExtension(double inches){
        if(inches > MAX_ARM_EXTENSION || inches < 0)
            return false;

        if(inches * Math.cos(Math.toRadians(targetAngle)) > MAX_HORIZONTAL_EXTENSION)
            return false;

        targetExtension = inches;
        return true;
    }
}
