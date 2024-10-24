package org.firstinspires.ftc.teamcode.hi;

public class MotionProfileController {
    private double maxSpeed;
    private double maxAcceleration;
    private double targetPosition;
    private double decelFactor = 2;
    private double currentSpeed = 0;

    public MotionProfileController(double maxSpeed, double maxAcceleration, double targetPosition) {
        this.maxSpeed = maxSpeed;
        this.maxAcceleration = maxAcceleration;
        this.targetPosition = targetPosition;
    }
    public void setTargetPosition(double newTargetPosition) {
        this.targetPosition = newTargetPosition;
        this.currentSpeed = 0;  // Reset speed when setting a new target
    }
    public void setAcceleration(double a){
        this.maxAcceleration = a;
    }
    public void setDecelFactor(double f){
        this.decelFactor = f;
    }

    public double calculate(double currentPosition) {
        double distanceToTarget = targetPosition - currentPosition;

        double targetSpeed = calculateTargetSpeed(distanceToTarget);

        // Adjust current speed to approach target speed gradually (smooth acceleration/deceleration)
        if (currentSpeed < targetSpeed) {
            currentSpeed = Math.min(currentSpeed + maxAcceleration, targetSpeed);
        } else if (currentSpeed > targetSpeed) {
            currentSpeed = Math.max(currentSpeed - maxAcceleration, targetSpeed);
        }

        // Return the current speed to set as motor power
        return currentSpeed;
    }
    private double calculateTargetSpeed(double distanceToTarget) {
        double brakingDistance = (maxSpeed * maxSpeed) / (maxAcceleration);

        // If we're far away, we can go at max speed
        if (distanceToTarget > brakingDistance) {
            return maxSpeed;
        } else {
            // Slow down as we approach the target (linear deceleration)
            return decelFactor * maxAcceleration;
        }
    }

}
