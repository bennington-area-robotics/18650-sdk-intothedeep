package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class PID {
    private final ElapsedTime timer = new ElapsedTime();
    double kP, kI, kD, kF;
    double i, maxI;
    double lastError;
    double tolerance;
    double minimum;
    Direction direction;
    double lastResult;

    public enum Direction {
        FORWARD, REVERSE
    }


    /**
     * Creates a PIDF controller which can be used to easily apply a control loop to almost anything.
     *
     * @param kP the proportional coefficient. This controls how much the magnitude of the error affects the output.
     * @param kI the integral coefficient. This controls how much the overall change in error affects the output.
     * @param kD the derivative coefficient. This controls how much the change in the error affects the output.
     * @param kF the feed-forward constant. This is a directly outputted constant which is added to the output.
     * @param maxI the maximum of the integral sum. This controls the maximum amount the integral calculation can affect the output.
     * @param tolerance the distance at which the controller considers having reached the target.
     */
    public PID(double kP, double kI, double kD, double kF, double maxI, double tolerance){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.maxI = maxI;
        this.direction = Direction.FORWARD;
        this.tolerance = tolerance;
    }

    /**
     * Creates a simple PF controller which can be used to easily apply a control loop to almost anything.
     *
     * @param kP the proportional coefficient. This controls how much the magnitude of the error affects the output.
     * @param kF the feed-forward constant. This is a directly outputted constant which is added to the output.
     * @param tolerance the distance at which the controller considers having reached the target.
     */
    public PID(double kP, double kF, double tolerance){
        this(kP, 0, 0, kF, 0, tolerance);
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
        if(Math.abs(currentError) > tolerance) {
            double timeChange = timer.milliseconds();
            timer.reset();

            double p = kP * currentError;

            i += kI * (currentError * (timeChange));

            if (i > maxI)
                i = maxI;
            else if (i < -maxI)
                i = -maxI;

            double d = kD * (currentError - lastError);

            lastError = currentError;

            double output;

            output = (p + i + d + kF);

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

    /**
     * @param kP the proportional coefficient. This controls how much the magnitude of the error affects the output.
     * @param kI the integral coefficient. This controls how much the overall change in error affects the output.
     * @param kD the derivative coefficient. This controls how much the change in the error affects the output.
     * @param kF the feed-forward constant. This is a directly outputted constant which is added to the output.
     * @param maxI the maximum of the integral sum. This controls the maximum amount the integral calculation can affect the output.
     */
    public void setConstants(double kP, double kI, double kD, double kF, double maxI){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.maxI = maxI;
    }

    public PID setTolerance(double tolerance){
        this.tolerance = tolerance;
        return this;
    }
}
