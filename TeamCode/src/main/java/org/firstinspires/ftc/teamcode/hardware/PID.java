package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class PID {
    private final ElapsedTime timer = new ElapsedTime();
    double kP, kI, kD;
    double i, maxI;
    double lastError;
    double tolerance;
    double minimum;

    /**
     * Creates a PID controller which can be used to easily apply to most things.
     *
     * @param kP the proportional coefficient. This controls how much the magnitude of the error affects the output.
     * @param kI the integral coefficient. This controls how much the overall change in error affects the output.
     * @param kD the derivative coefficient. This controls how much the change in the error affects the output.
     * @param maxI the maximum of the integral sum. This controls the maximum amount the integral calculation can affect the output.
     * @param minimum the minimum power level to output if outside of tolerance.
     */
    public PID(double kP, double kI, double kD, double maxI, double minimum){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.maxI = maxI;
        this.minimum = minimum;
    }

    /**
     * Calculates the output of the PID based on the error from the target. Note that errors and outputs are directional.
     * This means that a negative error input will yield a negative (or approaching negative) output value.
     *
     * @param currentError the current (directional) error value from the target.
     * @return the directional output power of the PID.
     */
    public double tick(double currentError){
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

            return Math.max(p + i + d, minimum);
        }else {
            return 0;
        }
    }

    /**
     * @param errorSupplier a function which supplies the current (directional) error of the motor.
     * @param outputConsumer a function which executes the necessary operation with the output (directionally).
     * @param tolerance the acceptable magnitude of error for the PID to have reached the target.
     * @return the time in milliseconds it took to reach the target.
     */
    public double bringToTarget(Supplier<Double> errorSupplier, Consumer<Double> outputConsumer, double tolerance){
        ElapsedTime time = new ElapsedTime();
        double error;
        while(Math.abs(error = errorSupplier.get()) > tolerance){
            outputConsumer.accept(tick(error));
        }

        return time.milliseconds();
    }

    /**
     * @param kP the proportional coefficient. This controls how much the magnitude of the error affects the output.
     * @param kI the integral coefficient. This controls how much the overall change in error affects the output.
     * @param kD the derivative coefficient. This controls how much the change in the error affects the output.
     * @param maxI the maximum of the integral sum. This controls the maximum amount the integral calculation can affect the output.
     */
    public void setConstants(double kP, double kI, double kD, double maxI){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.maxI = maxI;
    }

    public PID setTolerance(double tolerance){
        this.tolerance = tolerance;
        return this;
    }
}
