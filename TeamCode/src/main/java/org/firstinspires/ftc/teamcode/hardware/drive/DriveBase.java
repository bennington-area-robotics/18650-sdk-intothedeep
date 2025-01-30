package org.firstinspires.ftc.teamcode.hardware.drive;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveBase extends ConfiguredMecanumDrive {
    private double powerFactor = 1;

    public DriveBase(HardwareMap hardwareMap) {
        super(hardwareMap);
        setPoseEstimate(new Pose2d(0, 3 * 24, Math.toRadians(90)));
    }

    /**
     * Moves and turns the robot using general power modifiers in each direction/axis
     * @param x the power to move left/right with. Positive -> right, Negative -> left
     * @param y the power to move forward/back with. Positive -> forward, Negative -> backward
     * @param turn the power to turn with. Positive -> turn right, Negative -> turn left
     */
    public void move(double x, double y, double turn){
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the correct ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);
        double leftFront = (y + x + turn) / denominator;
        double leftRear = (y - x + turn) / denominator;
        double rightFront = (y - x - turn) / denominator;
        double rightRear = (y + x - turn) / denominator;

        setMotorPowers(leftFront * powerFactor, leftRear * powerFactor, rightRear * powerFactor, rightFront * powerFactor);
    }

    /**
     * Moves and turns the robot using general power modifiers in each direction/ axis
     * @param x the power to move left/ right with. Positive -> right, Negative -> left
     * @param y the power to move forward/ back with. Positive -> forward, Negative -> backward
     */
    public void move(double x, double y){
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the correct ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);
        double leftFront = (y + x) / denominator;
        double leftRear = (y - x) / denominator;
        double rightFront = (y - x) / denominator;
        double rightRear = (y + x) / denominator;

        setMotorPowers(leftFront * powerFactor, leftRear * powerFactor, rightRear * powerFactor, rightFront * powerFactor);
    }

    public Pose getPoseSimple(){
        return Pose.from(super.getPoseEstimate());
    }

    /**
     * Stops all motors. This is a shortcut method for <code>driveBase.setMotorPowers(0, 0, 0, 0)</code>`.
     */
    public void stop(){
        setMotorPowers(0,0,0,0);
    }

    public void setPowerFactor(double powerFactor){
        this.powerFactor = powerFactor;
    }

    public void followTrajectories(Trajectory... trajectories) {
        for (Trajectory trajectory : trajectories) {
            followTrajectory(trajectory);
        }
    }
}
