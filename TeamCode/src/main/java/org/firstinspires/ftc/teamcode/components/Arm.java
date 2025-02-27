package org.firstinspires.ftc.teamcode.components;

import androidx.annotation.FloatRange;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Direction;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.SmartEncoder;
import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.SmartTouchSensor;
import org.firstinspires.ftc.teamcode.hardware.controllers.BidirectionalPID;
import org.firstinspires.ftc.teamcode.hardware.controllers.PIDFConstants;
import org.firstinspires.ftc.teamcode.drive.roadrunner.util.Encoder;

import java.util.function.Consumer;

@Config
public class Arm {

    //todo split this into a wrapper for two separate wrappers, one wrapper controls the rotation base, the other the telescoping arm

    //<editor-fold desc="Config">
    public static float ARM_TICKS_PER_DEGREE = 65f; //this is a good estimate as of 1/24/2025

    public static float ARM_TICKS_PER_INCH = 190f;
    public static double MAX_ARM_EXTENSION = 37.0;

    public static double MAX_HORIZONTAL_EXTENSION = 38.0;

    public static double DELIVERY_EXTENSION = 38.0;
    public static double DELIVERY_ANGLE = 95.0;

    public static double COLLECTION_EXTENSION = 0.0;
    public static double COLLECTION_ANGLE = 0.0;
    public static double SPECIMEN_ANGLE = 55;

    public static double downwardKP = 0.000375, downwardKI = 0, downwardKD = 0.05, downwardKF = -0.175, downwardMaxI = 0;
    public static double upwardKP = 0.025, upwardKI = 0, upwardKD = 0.15, upwardKF = 0.325, upwardMaxI = 0;
    public static double extensionKP = 0.2, extensionKI, extensionKD, extensionKF = 0.15, extensionMaxI;
    public static double retractionKP = 0.1, retractionKI, retractionKD, retractionKF = -0.5, retractionMaxI;

    private final BidirectionalPID anglePid = new BidirectionalPID(
            PIDFConstants.of(upwardKP, upwardKI, upwardKD, upwardKF, upwardMaxI),
            PIDFConstants.of(downwardKP, downwardKI, downwardKD, downwardKF, downwardMaxI),
            0.75
    );

    private final BidirectionalPID extensionPid = new BidirectionalPID(
            PIDFConstants.of(extensionKP, extensionKI, extensionKD, extensionKF, extensionMaxI),
            PIDFConstants.of(retractionKP, retractionKI, retractionKD, retractionKF, retractionMaxI),
            0.4
    );
    //</editor-fold>

    private final SmartMotor angleMotorRight;
    private final SmartMotor angleMotorLeft;
    private final SmartMotor extensionMotor;

    private final SmartEncoder angleEncoder;
    public final SmartTouchSensor tiltLimitSensor;
    public final SmartTouchSensor extensionLimitSensor;

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

    public Arm(HardwareMap hardwareMap, String tiltMotorLeftName, String tiltMotorRightName, String extensionMotorName, String tiltSensorName, String extensionSensorName) {
        //<editor-fold desc="Hardware Config">
        this.angleMotorRight = Hardware.getMotor(tiltMotorRightName, true);
        this.angleMotorLeft = Hardware.getMotor(tiltMotorLeftName);
        this.extensionMotor = Hardware.getMotor(extensionMotorName);
        this.tiltLimitSensor = Hardware.getTouchSensor(tiltSensorName);
        this.extensionLimitSensor = Hardware.getTouchSensor(extensionSensorName);
        this.angleEncoder = angleMotorRight.getEncoder();

        this.extensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.angleMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.angleMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        this.angleMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.angleMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.angleMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.angleMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.angleEncoder.setDirection(Direction.FORWARD);
        //</editor-fold>

        resetExtension();
        resetAngle();

        targetAngle = getAngle();
        targetExtension = getExtension();
    }

    /**
     * Get the current angle of the arm. This is relative to the base, at 0 the arm is horizontal, and at 90 the arm is vertical.
     * Uses should be able to handle angles past 90 degrees, since the motor will not always land at exactly 90.
     *
     * @return the angle of the arm relative to the base.
     */
    public double getAngle(){
        return (angleEncoder.getPosition() - tickOffsetToZero) / ARM_TICKS_PER_DEGREE;
    }

    /**
     * Get the current extension of the end of the arm past the minimum extension (fully retracted).
     *
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
        tickOffsetToZero = angleEncoder.getPosition();
    }

    /**
     * Sets the current extension as the target and zero position.
     */
    public void resetExtension(){
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        targetExtension = 0;
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

    /**
     * Moves arm to collection pose.
     */
    public void collectionPosition(){
        if(getExtension() - COLLECTION_EXTENSION < 1.5){
            setTargetExtension(COLLECTION_EXTENSION);
            setTargetAngle(0);
        }else {
            double inchesPerDegree = (getAngle() - COLLECTION_ANGLE) / (getExtension() - COLLECTION_EXTENSION);

            double startAngle = getAngle();
            double startExtension = getExtension();

            setTargetExtension(COLLECTION_EXTENSION);
            runningMacro = (arm -> {
                if (Math.abs(COLLECTION_ANGLE - arm.getAngle()) < 10) {
                    arm.setTargetAngleIgnoreMacro(COLLECTION_ANGLE);
                    arm.setTargetExtension(arm.getExtension());
                    arm.runningMacro = null;
                } else if (getExtension() - COLLECTION_EXTENSION < 1.5) {
                    arm.setTargetAngleIgnoreMacro(COLLECTION_ANGLE);
                    arm.setTargetExtension(DELIVERY_EXTENSION);
                } else {
                    double targetAngle = inchesPerDegree * (arm.getExtension() - COLLECTION_EXTENSION) + COLLECTION_ANGLE;
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
        if(tiltLimitSensor.isPressed())
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
        anglePid.reverseSet.set(downwardKP, downwardKI, downwardKD,downwardKF, downwardMaxI);
        anglePid.forwardSet.set(upwardKP, upwardKI, upwardKD, upwardKF, upwardMaxI);

        lastAnglePower = anglePid.calc(targetAngle - getAngle());

        return lastAnglePower;
    }

    public double getLastAnglePower(){
        return lastAnglePower;
    }

    double lastExtensionPower;
    private double getExtensionPower(){
        extensionPid.forwardSet.set(extensionKP, extensionKI, extensionKD, extensionKF, extensionMaxI);
        extensionPid.reverseSet.set(retractionKP, retractionKI, retractionKD, retractionKF, retractionMaxI);

        lastExtensionPower = extensionPid.calc(targetExtension - getExtension());

        return lastExtensionPower;
    }

    public double getLastExtensionPower(){
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
