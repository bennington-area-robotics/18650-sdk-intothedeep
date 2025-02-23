package org.firstinspires.ftc.teamcode.hardware.controllers;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID implements ControlAlg {
    double i, lastError, tolerance, minimum, lastResult;

    public final Constants constants;
    private final ElapsedTime timer = new ElapsedTime();
    Direction direction;

    public enum Direction {
        FORWARD, REVERSE
    }

    /**
     * Creates a PIDF controller which can be used to easily apply a control loop to almost anything.
     *
     * @param tolerance the distance at which the controller considers having reached the target.
     */
    public PID(Constants constants, double tolerance){
        this.constants = constants;
        this.direction = Direction.FORWARD;
        this.tolerance = tolerance;
    }

    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    public Direction getDirection() {
        return direction;
    }

    /**
     * Calculates the output of the PID based on the error from the target. Note that errors and outputs are directional.
     * This means that a negative error input will yield a negative (or approaching negative) output value.
     *
     * @param currentError the current (directional) error value from the target.
     * @return the directional output power of the PID.
     */
    public double calc(double currentError){
        return calc(currentError, constants);
    }

    /**
     * Calculates the output of the PID based on the error from the target. Note that errors and outputs are directional.
     * This means that a negative error input will yield a negative (or approaching negative) output value.
     *
     * @param currentError the current (directional) error value from the target.
     * @return the directional output power of the PID.
     */
    double calc(double currentError, Constants constants){
        if(Math.abs(currentError) > tolerance) {
            double timeChange = timer.milliseconds();
            timer.reset();

            double p = constants.p * currentError;

            i += constants.i * (currentError * (timeChange));

            if (i > constants.maxI)
                i = constants.maxI;
            else if (i < -constants.maxI)
                i = -constants.maxI;

            double d = constants.d * (currentError - lastError);

            lastError = currentError;

            double output;

            output = (p + i + d + constants.f);

            if(direction == Direction.REVERSE) {
                output = -output;
            }

            lastResult = output;

        }else {
            lastResult = 0;
        }
        return lastResult;
    }

    public double result(){
        return lastResult;
    }

    public PID setTolerance(double tolerance){
        this.tolerance = tolerance;
        return this;
    }
}
