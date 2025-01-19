package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.FloatRange;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.OpModeCore;
import org.firstinspires.ftc.teamcode.util.Encoder;

@Config
public class Arm {
    //config
        public static float ARM_TICKS_PER_DEGREE = 70f;

        public static float ARM_TICKS_PER_INCH = 190f; //TODO change this to a real number
        public static double MAX_ARM_EXTENSION = 38.0;

        public static double MAX_HORIZONTAL_EXTENSION = 38.0;
    //config

    private final DcMotorEx angleMotorRight;
    private final DcMotorEx angleMotorLeft;
    private final DcMotorEx extensionMotor;

    private final Encoder angleEncoder;

    /**
     * Target extension of the arm in inches past the minimum extension (not extended at all)
     */
    private double targetExtension = 0;

    /**
     * Target angle of the arm in degrees relative to the base. 0 is horizontal, while 90 is vertical.
     */
    private double targetAngle = 0;

    private double tickOffsetToZero;

    //todo these need actual trained values
    public static double angleKP = 0.1, angleKI, angleKD, angleMaxI;
    public static double extensionKP = 0.1, extensionKI, extensionKD, extensionMaxI;

    public static double GRAVITY_COMPENSATION = 0.2;

    private final PID.FeedForwardFunction angleFFFunction = new PID.FeedForwardFunction() {
        @Override
        public Double apply(Double currentError, Double currentPosition) {
            // Calculate gravity compensation
            // Maximum at horizontal (0°), minimum at vertical (90°)
            double gravityCompensation = Math.cos(Math.toRadians(currentPosition)) * GRAVITY_COMPENSATION;
            
            // Always apply gravity compensation upward, regardless of direction of movement
            return gravityCompensation;
        }
    };

    private final PID anglePID = new PID(this::getAngle, angleKP, angleKI, angleKD, angleMaxI).setTolerance(0.01).addFeedForwardFunction(angleFFFunction);
    private final PID extensionPID = new PID(this::getExtension, extensionKP, extensionKI, extensionKD, extensionMaxI).setTolerance(0.01);

    public Arm(HardwareMap hardwareMap, String tiltMotorLeftName, String tiltMotorRightName, String extensionMotorName) {
        this.angleMotorRight = hardwareMap.get(DcMotorEx.class, tiltMotorRightName);
        this.angleMotorLeft = hardwareMap.get(DcMotorEx.class, tiltMotorLeftName);
        this.extensionMotor = hardwareMap.get(DcMotorEx.class, extensionMotorName);
        this.angleEncoder = new Encoder(angleMotorRight);
        this.extensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.angleMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.angleMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        this.angleMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.angleMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.angleEncoder.setDirection(Encoder.Direction.FORWARD);

        resetAngle();
        resetExtension();

        OpModeCore.getTelemetry().addData("Current Arm Angle", this::getAngle);
        OpModeCore.getTelemetry().addData("Target Arm Angle", this::getTargetAngle);
        OpModeCore.getTelemetry().addData("Current Arm Extension", this::getExtension);
        OpModeCore.getTelemetry().addData("Target Arm Extension", this::getTargetExtension);
        OpModeCore.getTelemetry().addData("Encoder Position", () -> angleEncoder.getCurrentPosition() - tickOffsetToZero);
    }

    /**
     * Get the current angle of the arm. This is relative to the base, at 0 the arm is horizontal, and at 90 the arm is vertical.
     * Uses should be able to handle angles past 90 degrees, since the motor will not always land at exactly 90.
     *
     * @apiNote This method is relatively costly due to reading motor positions, avoid calling more than necessary.
     * @return the angle of the arm relative to the base.
     */
    public double getAngle(){
        return ( angleEncoder.getCurrentPosition() - tickOffsetToZero) / ARM_TICKS_PER_DEGREE;
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
        //do not set the target to a degree outside the desired range of motion
        if(degrees < 0 || degrees > 90)
            return false;

        //do not set the target to a degree that will cause the arm to move outside the extension bounds.
        if(targetExtension * Math.cos(Math.toRadians(degrees)) > MAX_HORIZONTAL_EXTENSION)
            return false;

        targetAngle = degrees;
        return true;
    }

    /**
     * Checks if the passed target extension in inches is valid, then sets the target extension if so.
     * A target extension is valid if the arm will not extend past the horizontal extension limit when at the current target angle, and that target extension.
     *
     * @param inches the target extension in inches.
     * @return whether the operation was successful (whether it passed the checks).
     */
    public boolean setTargetExtension(double inches){
        if(inches * Math.cos(Math.toRadians(targetAngle)) > MAX_HORIZONTAL_EXTENSION || inches > MAX_ARM_EXTENSION || inches < 0)
            return false;

        targetExtension = inches;
        return true;
    }

    /**
     * Sets the current angle as the target and zero position.
     */
    public void resetAngle(){
        tickOffsetToZero = angleEncoder.getCurrentPosition();
        targetAngle = 0;
    }

    /**
     * Sets the current extension as the target and zero position.
     */
    public void resetExtension(){
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        targetExtension = 0;
        extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private double getCollectorExtension(){
        if(OpModeCore.getCollector().isWristUp())
            return Collector.LENGTH;
        return 0;
    }


    /*todo if this method is not performant enough it could be made better:
        make it automatically detect once it is close enough to its target (simply tolerance),
        then only sample the motors one in every so many calls. every other call should just maintain the old powers.
     */
    /**
     * Runs a cycle on the PIDF control loop for the arm.
     * This method should be called once per OpMode cycle to maintain the arm's position when at target,
     * or adjust the arm's position when not at target. This controls both extension and retraction
     */
    public void tickPIDF(){
        anglePID.setConstants(angleKP, angleKI, angleKD, angleMaxI);
        extensionPID.setConstants(extensionKP, extensionKI, extensionKD, extensionMaxI);

        double anglePower = anglePID.tick(targetAngle - getAngle());

        angleMotorRight.setPower(anglePower);
        angleMotorLeft.setPower(anglePower);
        extensionMotor.setPower(extensionPID.tick(targetExtension - getExtension()));

    }
}
