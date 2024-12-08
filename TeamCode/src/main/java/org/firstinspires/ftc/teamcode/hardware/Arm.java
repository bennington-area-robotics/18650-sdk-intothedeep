package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.Configuration.ARM_TICKS_PER_DEGREE;
import static org.firstinspires.ftc.teamcode.Configuration.ARM_TICKS_PER_INCH;
import static org.firstinspires.ftc.teamcode.Configuration.MAX_ARM_EXTENSION;

import androidx.annotation.FloatRange;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Arm {
    DcMotor angleMotor, extensionMotor;

    /**
     * Target extension of the arm in inches past the minimum extension (not extended at all)
     */
    private double targetExtension;

    /**
     * Target angle of the arm in degrees relative to the base. 0 is horizontal, while 90 is vertical.
     */
    private double targetAngle;

    //todo these need actual trained values
    public static double angleKP, angleKI, angleKD, angleMaxI;
    public static double extensionKP, extensionKI, extensionKD, extensionMaxI;

    private final PID anglePID = new PID(angleKP, angleKI, angleKD, angleMaxI);
    private final PID extensionPID = new PID(extensionKP, extensionKI, extensionKD, extensionMaxI);

    public Arm(HardwareMap hardwareMap, String tiltMotorName, String extensionMotorName) {
        this.angleMotor = hardwareMap.get(DcMotor.class, tiltMotorName);
        this.extensionMotor = hardwareMap.get(DcMotor.class, extensionMotorName);
    }

    /**
     * Get the current angle of the arm. This is relative to the base, at 0 the arm is horizontal, and at 90 the arm is vertical.
     * Uses should be able to handle angles past 90 degrees, since the motor will not always land at exactly 90.
     *
     * @apiNote This method is relatively costly due to reading motor positions, avoid calling more than necessary.
     * @return the angle of the arm relative to the base.
     */
    public double getAngle(){
        return angleMotor.getCurrentPosition() * ARM_TICKS_PER_DEGREE;
    }

    /**
     * Get the current extension of the end of the arm past the minimum extension (fully retracted).
     *
     * @apiNote This method is relatively costly due to reading motor positions, avoid calling more than necessary.
     * @return the extension of the end of the arm.
     */
    public double getExtension(){
        return extensionMotor.getCurrentPosition() * ARM_TICKS_PER_INCH;
    }

    public double getTargetExtension(){
        return targetExtension;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    /**
     * Checks if the passed target angle in inches is valid, then sets the target extension if so.
     * A target angle is valid if the arm will not extend past the 20 inch horizontal extension limit when at that target angle, and the current target extension.
     *
     * @param degrees the target angle in degrees.
     * @return whether the operation was successful (whether it passed the checks).
     */
    public boolean setTargetAngle(@FloatRange(from=0, to=90) Double degrees){
        //do not set the target to a degree outside the desired range of motion
        if(degrees < 0 || degrees > 90)
            return false;

        //do not set the target to a degree that will cause the arm to move outside the extension bounds.
        if(targetExtension * Math.cos(Math.toRadians(degrees)) > MAX_ARM_EXTENSION)
            return false;

        targetAngle = degrees;
        return true;
    }

    /**
     * Checks if the passed target extension in inches is valid, then sets the target extension if so.
     * A target extension is valid if the arm will not extend past the 20 inch limit when at the current target angle, and that target extension.
     *
     * @param inches the target extension in inches.
     * @return whether the operation was successful (whether it passed the checks).
     */
    public boolean setTargetExtension(Double inches){
        if(inches * Math.cos(Math.toRadians(targetAngle)) > MAX_ARM_EXTENSION)
            return false;

        targetExtension = inches;
        return true;
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
        angleMotor.setPower(anglePID.tick(targetAngle - getAngle()));
        extensionMotor.setPower(extensionPID.tick(targetAngle - getAngle()));
    }
}
