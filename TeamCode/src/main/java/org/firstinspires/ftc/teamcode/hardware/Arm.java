package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.FloatRange;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.OpModeCore;

@Config
public class Arm {
    //config
        private static final float ARM_GEAR_RATIO = 86.0f / 28.0f;
        private static final float ARM_TICKS_PER_DEGREE_AT_MOTOR_OUTPUT = 288.0f/360.0f;

        public static float ARM_TICKS_PER_DEGREE = 12.5f;

        public static float ARM_TICKS_PER_INCH = 190f; //TODO change this to a real number
        public static double MAX_ARM_EXTENSION = 38.0;

        public static double MAX_HORIZONTAL_EXTENSION = 38.0;
    //config

    private final DcMotor angleMotorRight, angleMotorLeft, extensionMotor;

    /**
     * Target extension of the arm in inches past the minimum extension (not extended at all)
     */
    private double targetExtension = 0;

    /**
     * Target angle of the arm in degrees relative to the base. 0 is horizontal, while 90 is vertical.
     */
    private double targetAngle = 0;

    //todo these need actual trained values
    public static double angleKP = 0.1, angleKI, angleKD, angleMaxI;
    public static double extensionKP = 0.1, extensionKI, extensionKD, extensionMaxI;

    private final PID anglePID = new PID(angleKP, angleKI, angleKD, angleMaxI).setTolerance(0.01);
    private final PID extensionPID = new PID(extensionKP, extensionKI, extensionKD, extensionMaxI).setTolerance(0.01);

    public Arm(HardwareMap hardwareMap, String tiltMotorLeftName, String tiltMotorRightName, String extensionMotorName) {
        this.angleMotorRight = hardwareMap.get(DcMotor.class, tiltMotorRightName);
        this.angleMotorLeft = hardwareMap.get(DcMotor.class, tiltMotorLeftName);
        this.extensionMotor = hardwareMap.get(DcMotor.class, extensionMotorName);
        this.extensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.angleMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.angleMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetAngle();
        resetExtension();

        OpModeCore.getTelemetry().addData("Current Arm Angle", this::getAngle);
        OpModeCore.getTelemetry().addData("Target Arm Angle", this::getTargetAngle);
        OpModeCore.getTelemetry().addData("Current Arm Extension", this::getExtension);
        OpModeCore.getTelemetry().addData("Target Arm Extension", this::getTargetExtension);
    }

    /**
     * Get the current angle of the arm. This is relative to the base, at 0 the arm is horizontal, and at 90 the arm is vertical.
     * Uses should be able to handle angles past 90 degrees, since the motor will not always land at exactly 90.
     *
     * @apiNote This method is relatively costly due to reading motor positions, avoid calling more than necessary.
     * @return the angle of the arm relative to the base.
     */
    public double getAngle(){
        return angleMotorRight.getCurrentPosition() / ARM_TICKS_PER_DEGREE;
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
        angleMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        targetAngle = 0;
        angleMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Sets the current extension as the target and zero position.
     */
    public void resetExtension(){
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        targetExtension = 0;
        extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

//    /**
//     * Sets the target extension to the desired amount if below the maximum extension, and modifies the target angle to allow for the desired extension.
//     * @param inches the desired extension amount in inches.
//     * @return false if inches was past the possible extension, otherwise true;
//     */
//    public boolean setTargetExtensionDynamic(double inches){
//        if(setTargetExtension(inches))
//            return true;
//        else{
//
//        }
//    }

//    public void setTargetAngleDynamic(double inches){
//
//    }

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
