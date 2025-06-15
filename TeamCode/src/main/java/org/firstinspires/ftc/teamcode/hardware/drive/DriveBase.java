package org.firstinspires.ftc.teamcode.hardware.drive;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.controllers.PID;

@Config
public class DriveBase extends ConfiguredMecanumDrive {
    public static float HIGH_TRANSLATIONAL_VELOCITY_MULTIPLIER = 30f;
    public static float HIGH_HEADING_VELOCITY_MULTIPLIER = 2f;
    public static float LOW_TRANSLATIONAL_VELOCITY_MULTIPLIER = 20f;
    public static float LOW_HEADING_VELOCITY_MULTIPLIER = 2f;

    public static int inputExponent = 2;
    private double powerFactor = 1;
    public static double kP = 3, kI = 0.001, kD = 0.1, kF = 0, maxKI =0;
    public static PID rotationalPID = new PID(kP, kI, kD, kI, maxKI, 0.5);
    private boolean lowPowerMode = false;
    public static double lowPowerMax = 0.5;
    public static double minPowerThreshold = 0.1;

    private double lastX = 0;
    private double lastY = 0;
    private double lastTurn = 0;
    public static double maxTranslationAccel = 0.8; // For X and Y movement (0.05-0.2 range)
    public static double maxRotationAccel = 1;    // For turning (typically lower than translation)
    public static double directionChangeBoost = 4.0;
    private long lastLoopTimeMs = System.currentTimeMillis(); // Track actual loop time

    public DriveBase(HardwareMap hardwareMap) {
        super(hardwareMap);
        setPoseEstimate(new Pose2d(0, 63, Math.toRadians(-90)));
    }

    /**
     * Moves and turns the robot using general power modifiers in each direction/axis
     * @param x the power to move left/right with. Positive -> right, Negative -> left
     * @param y the power to move forward/back with. Positive -> forward, Negative -> backward
     * @param turn the power to turn with. Positive -> turn right, Negative -> turn left
     */
    public void moveUsingRR(double x, double y, double turn){
        float TRANSLATIONAL_VELOCITY_MULTIPLIER, HEADING_VELOCITY_MULTIPLIER;

        if(lowPowerMode){
            TRANSLATIONAL_VELOCITY_MULTIPLIER = LOW_TRANSLATIONAL_VELOCITY_MULTIPLIER;
            HEADING_VELOCITY_MULTIPLIER = LOW_HEADING_VELOCITY_MULTIPLIER;
        }else{
            TRANSLATIONAL_VELOCITY_MULTIPLIER = HIGH_TRANSLATIONAL_VELOCITY_MULTIPLIER;
            HEADING_VELOCITY_MULTIPLIER = HIGH_HEADING_VELOCITY_MULTIPLIER;
        }

        setDriveSignal(new DriveSignal(
                new Pose2d(
                TRANSLATIONAL_VELOCITY_MULTIPLIER * y,
                TRANSLATIONAL_VELOCITY_MULTIPLIER * x,
                HEADING_VELOCITY_MULTIPLIER * -turn
                )
        ));
    }

    public void moveWithAcceleration(double targetX, double targetY, double targetTurn) {
        // Apply quadratic scaling to inputs (maintains sign but makes control more precise)
        targetX = Math.signum(targetX) * Math.pow(Math.abs(targetX), inputExponent);
        targetY = Math.signum(targetY) * Math.pow(Math.abs(targetY), inputExponent);
        targetTurn = Math.signum(targetTurn) * Math.pow(Math.abs(targetTurn), inputExponent);

        boolean isZeroInputTranslation = Math.abs(targetX) < 0.05 && Math.abs(targetY) < 0.05;
        boolean isZeroInputRotation = Math.abs(targetTurn) < 0.05;

        if (isZeroInputRotation) {
            lastTurn = 0;
            if (isZeroInputTranslation){
                // Either stop immediately or decelerate more quickly
                lastX = 0;
                lastY = 0;
                lastTurn = 0;
                moveUsingPower(0, 0, 0);
                return;
            }



        }
        // Calculate actual loop time
        long currentTimeMs = System.currentTimeMillis();
        double actualLoopTimeSeconds = (currentTimeMs - lastLoopTimeMs) / 1000.0;
        lastLoopTimeMs = currentTimeMs;


        actualLoopTimeSeconds = Math.min(actualLoopTimeSeconds, 0.1);

        // Apply acceleration limits
        double deltaX = targetX - lastX;
        double deltaY = targetY - lastY;
        double deltaTurn = targetTurn - lastTurn;

        // Check for direction changes (sign changes) and apply boost if needed
        double actualTransAccelX = maxTranslationAccel;
        double actualTransAccelY = maxTranslationAccel;
        double actualRotAccel = maxRotationAccel;


        if (lastX * targetX < 0 && Math.abs(targetX) > 0.1) { // X direction change
            actualTransAccelX *= directionChangeBoost;
        }

        if (lastY * targetY < 0 && Math.abs(targetY) > 0.1) { // Y direction change
            actualTransAccelY *= directionChangeBoost;
        }

        if (lastTurn * targetTurn < 0 && Math.abs(targetTurn) > 0.1) { // Turn direction change
            actualRotAccel *= directionChangeBoost;
        }


        if ((Math.abs(targetX) > 0.3 && Math.abs(lastY) > 0.3 && Math.abs(lastX) < 0.2) ||
                (Math.abs(targetY) > 0.3 && Math.abs(lastX) > 0.3 && Math.abs(lastY) < 0.2)) {

            // When switching primary movement axes, rapidly reduce the old axis
            if (Math.abs(lastX) > Math.abs(targetX) && Math.abs(lastX) > 0.2) {
                // Rapidly reduce X when switching to Y
                lastX *= 0.5; // Cut the current X velocity in half immediately
            }

            if (Math.abs(lastY) > Math.abs(targetY) && Math.abs(lastY) > 0.2) {
                // Rapidly reduce Y when switching to X
                lastY *= 0.5; // Cut the current Y velocity in half immediately
            }

            // Also boost acceleration in the new direction
            actualTransAccelX *= 1.5;
            actualTransAccelY *= 1.5;
        }

        // Limit the changes based on max acceleration and actual loop time
        double maxXDelta = actualTransAccelX * actualLoopTimeSeconds;
        double maxYDelta = actualTransAccelY * actualLoopTimeSeconds;
        double maxRotDelta = actualRotAccel * actualLoopTimeSeconds;

        // Clamp the changes to the maximum allowed change
        if (Math.abs(deltaX) > maxXDelta) {
            deltaX = Math.signum(deltaX) * maxXDelta;
        }

        if (Math.abs(deltaY) > maxYDelta) {
            deltaY = Math.signum(deltaY) * maxYDelta;
        }

        if (Math.abs(deltaTurn) > maxRotDelta) {
            deltaTurn = Math.signum(deltaTurn) * maxRotDelta;
        }

        // Calculate new values with limited acceleration
        double newX = lastX + deltaX;
        double newY = lastY + deltaY;
        double newTurn = lastTurn + deltaTurn;

        // Update last values for next iteration
        lastX = newX;
        lastY = newY;
        lastTurn = newTurn;

        // Call the original movement method with the acceleration-limited values
        moveUsingPower(newX, newY, newTurn);
    }

    // Methods to set the acceleration limits
    public void setMaxTranslationAcceleration(double acceleration) {
        maxTranslationAccel = Math.abs(acceleration); // Ensure positive value
    }

    public void setMaxRotationAcceleration(double acceleration) {
        maxRotationAccel = Math.abs(acceleration); // Ensure positive value
    }

    public void setAccelerationLimits(double translationAccel, double rotationAccel) {
        maxTranslationAccel = Math.abs(translationAccel);
        maxRotationAccel = Math.abs(rotationAccel);
    }


    public void resetAcceleration() {
        lastX = 0;
        lastY = 0;
        lastTurn = 0;
    }

    public void moveUsingPower(double x, double y, double turn){
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the correct ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);
        double leftFront = ((y - x + turn) / denominator) * powerFactor;
        double leftRear = ((y + x + turn) / denominator) * powerFactor;
        double rightFront = ((y + x - turn) / denominator) * powerFactor;
        double rightRear = ((y - x - turn) / denominator) * powerFactor;

        if (lowPowerMode){
            leftFront = Math.signum(leftFront) * Math.min(lowPowerMax, Math.abs(leftFront));
            leftRear = Math.signum(leftRear) * Math.min(lowPowerMax, Math.abs(leftRear));
            rightRear = Math.signum(rightRear) * Math.min(lowPowerMax, Math.abs(rightRear));
            rightFront = Math.signum(rightFront) * Math.min(lowPowerMax, Math.abs(rightFront));
        }

        leftFront = Math.signum(leftFront) * Math.max(minPowerThreshold, Math.abs(leftFront));
        leftRear = Math.signum(leftRear) * Math.max(minPowerThreshold, Math.abs(leftRear));
        rightRear = Math.signum(rightRear) * Math.max(minPowerThreshold, Math.abs(rightRear));
        rightFront = Math.signum(rightFront) * Math.max(minPowerThreshold, Math.abs(rightFront));

        setMotorPowers(leftFront, leftRear, rightRear, rightFront);
    }

    public Pose getPoseSimple(){
        updatePoseEstimate();
        return Pose.from(super.getPoseEstimate());
    }

    /**
     * Stops all motors. This is a shortcut method for <code>driveBase.setMotorPowers(0, 0, 0, 0)</code>`.
     */
    public void stop(){
        setMotorPowers(0,0,0,0);
    }
    public void squareUp(){

        while (Math.abs(this.getPoseSimple().heading()) > 0.5){
            rotationalPID.setConstants(kP, kI, kD, kF, maxKI);
            double power =  rotationalPID.calc(this.getPoseSimple().heading());
            setMotorPowers(power, power, -power, -power);

        }
        setMotorPowers(0, 0, 0, 0);

    }
    public void setPowerFactor(double powerFactor){
        this.powerFactor = powerFactor;
    }
    public void setLowPowerMode (boolean lowPowerMode){
        this.lowPowerMode = lowPowerMode;
    }

    public void followTrajectories(Trajectory... trajectories) {
        for (Trajectory trajectory : trajectories) {
            followTrajectory(trajectory);
        }
    }
}
