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
        public static float ARM_TICKS_PER_DEGREE = 70f;

        public static float ARM_TICKS_PER_INCH = 190f;
        public static double MAX_ARM_EXTENSION = 38.0;

        public static double MAX_HORIZONTAL_EXTENSION = 38.0;

        public static double DELIVERY_EXTENSION = 38.0;
        public static double DELIVERY_ANGLE = 90.0;

        public static double COLLECTION_EXTENSION = 0.0;
        public static double COLLECTION_ANGLE = 0.0;
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
    public static double angleKP = 0.02, angleKI = 0.00001, angleKD = 0.2, angleMaxI = 0.09;
    public static double extensionKP = 0.1, extensionKI, extensionKD, extensionMaxI;

    private final PID anglePID = new PID(angleKP, angleKI, angleKD, angleMaxI).setTolerance(0.01);
    private final PID extensionPID = new PID(extensionKP, extensionKI, extensionKD, extensionMaxI).setTolerance(0.01);


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

        resetAngle();
        resetExtension();

        targetAngle = getAngle();
        targetExtension = getExtension();

        OpModeCore.getTelemetry().addData("Current Arm Angle", this::getCachedAngle);
        OpModeCore.getTelemetry().addData("Target Arm Angle", this::getTargetAngle);
        OpModeCore.getTelemetry().addData("Current Arm Extension", this::getCachedExtension);
        OpModeCore.getTelemetry().addData("Target Arm Extension", this::getTargetExtension);
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
        return extensionMotor.getCurrentPosition() / ARM_TICKS_PER_INCH;
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
    public boolean setTargetAngle(@FloatRange(from=0, to=90) double degrees){
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
    private boolean setTargetAngleIgnoreMacro(@FloatRange(from=0, to=90) double degrees){
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

    /**
     * Moves arm to collection pose.
     */
    public void collectionPosition(){
        double anglePerExtension = (getCachedAngle() - COLLECTION_ANGLE) / (getCachedExtension() - COLLECTION_EXTENSION);
        double startAngle = getCachedAngle();
        double startExtension = getCachedExtension();

        setTargetExtension(COLLECTION_EXTENSION);
        runningMacro = (arm -> {
            if(Math.abs(arm.getTargetAngle() - arm.getCachedAngle()) < 0.5){
                arm.runningMacro = null;
            }else {
                arm.setTargetAngleIgnoreMacro(startAngle + anglePerExtension * (arm.getCachedExtension() + startExtension));
            }
        });
    }

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

        if(runningMacro != null){
            runningMacro.accept(this);
        }

        tickPIDF();
    }

    /**
     * Runs a cycle on the PIDF control loop for the arm.
     */
    private void tickPIDF(){
        anglePID.setConstants(angleKP, angleKI, angleKD, angleMaxI);
        extensionPID.setConstants(extensionKP, extensionKI, extensionKD, extensionMaxI);

        double anglePower = anglePID.tick(targetAngle - getCachedAngle());
        angleMotorRight.setPower(anglePower);
        angleMotorLeft.setPower(anglePower);
        extensionMotor.setPower(extensionPID.tick(targetExtension - getCachedAngle()));
    }

    public boolean isValidAngle(double degrees){
        //do not set the target to a degree outside the desired range of motion
        if(degrees < 0 || degrees > 90)
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
