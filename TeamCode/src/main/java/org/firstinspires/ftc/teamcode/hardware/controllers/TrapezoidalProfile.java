package org.firstinspires.ftc.teamcode.hardware.controllers;


public class TrapezoidalProfile {
    private double maxVelocity;      // Maximum velocity in ticks per second
    private double maxAcceleration;  // Maximum acceleration in ticks per second^2

    // Current profile state
    private double startPosition;    // Starting position in ticks
    private double targetPosition;   // Target position in ticks
    private double profileStartTime; // Time when profile started
    private double totalTime;        // Total time profile will take

    public TrapezoidalProfile(double maxVel, double maxAccel) {
        this.maxVelocity = maxVel;
        this.maxAcceleration = maxAccel;
    }

    public void setNewTarget(double currentPos, double targetPos, double currentTime) {
        startPosition = currentPos;
        targetPosition = targetPos;
        profileStartTime = currentTime;

        // Calculate total distance to move
        double distance = Math.abs(targetPosition - startPosition);

        // Calculate time needed to accelerate to max velocity
        double timeToMaxVel = maxVelocity / maxAcceleration;

        // Calculate distance covered during acceleration
        double accelDistance = 0.5 * maxAcceleration * timeToMaxVel * timeToMaxVel;

        // If we can't reach max velocity (triangle profile)
        if (accelDistance * 2 > distance) {
            // Time to reach peak velocity (which will be lower than max velocity)
            timeToMaxVel = Math.sqrt(distance / maxAcceleration);
            totalTime = timeToMaxVel * 2;
        } else {
            // Trapezoidal profile - we can reach max velocity
            double cruiseDistance = distance - (accelDistance * 2);
            double cruiseTime = cruiseDistance / maxVelocity;
            totalTime = (timeToMaxVel * 2) + cruiseTime;
        }
    }

    public double calculate(double currentTime) {
        double elapsedTime = currentTime - profileStartTime;

        // Clamp elapsed time to total profile time
        elapsedTime = Math.min(elapsedTime, totalTime);

        // Calculate desired position along profile
        double direction = Math.signum(targetPosition - startPosition);
        double distance = Math.abs(targetPosition - startPosition);

        if (elapsedTime < totalTime / 2) {
            // Accelerating phase
            return startPosition + direction * (0.5 * maxAcceleration * elapsedTime * elapsedTime);
        } else {
            // Decelerating phase
            double timeFromEnd = totalTime - elapsedTime;
            return targetPosition - direction * (0.5 * maxAcceleration * timeFromEnd * timeFromEnd);
        }
    }
}