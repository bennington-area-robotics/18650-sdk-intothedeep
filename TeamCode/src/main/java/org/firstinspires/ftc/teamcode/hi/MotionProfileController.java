package org.firstinspires.ftc.teamcode.hi;

public class MotionProfileController {
    private double maxSpeed;
    private double maxAcceleration;
    private double targetPosition;

    public MotionProfileController(double maxSpeed, double maxAcceleration, double targetPosition) {
        this.maxSpeed = maxSpeed;
        this.maxAcceleration = maxAcceleration;
        this.targetPosition = targetPosition;
    }
    public void setTarget(int target){
        this.targetPosition = target;
    }
    public void setAcceleration(double a){
        this.maxAcceleration = a;
    }

    public double calculate(double currentPosition, double currentSpeed) {
        double distanceToTarget = targetPosition - currentPosition;

        // Accelerate if speed is less than max and we're far from the target
        if (Math.abs(distanceToTarget) > maxSpeed * maxSpeed / (2 * maxAcceleration)) {
            return Math.min(currentSpeed + maxAcceleration, maxSpeed);  // Accelerating
        } else {
            // Decelerate as we approach the target
            return Math.max(currentSpeed - maxAcceleration, 0);  // Decelerating
        }
    }
}
