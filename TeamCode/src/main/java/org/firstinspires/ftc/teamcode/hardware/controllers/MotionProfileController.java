package org.firstinspires.ftc.teamcode.hardware.controllers;

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
        double brakingDistance = 2* (maxSpeed * maxSpeed) / (2* maxAcceleration);
        double targetSpeed = maxSpeed;
        if (distanceToTarget <= brakingDistance){
            targetSpeed = 0;
        }

        // Adjust current speed to approach target speed gradually (smooth acceleration/deceleration)
        if (currentSpeed < targetSpeed) {
            currentSpeed = Math.min(currentSpeed + maxAcceleration, targetSpeed);
        } else if (currentSpeed > targetSpeed) {
            currentSpeed = Math.max(currentSpeed - 2 * maxAcceleration, targetSpeed);
        }

        // Return the current speed to set as motor power
        return currentSpeed;
    }


}
