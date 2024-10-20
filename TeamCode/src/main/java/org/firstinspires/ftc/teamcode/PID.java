package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;


public class PID {

    private ElapsedTime timer = new ElapsedTime();
    private double integralSum;
    private double lastError;
    private double p, i, d, f;

    /**
     * Creates a new PID function using default PIDF variables
     */
    public PID(){
        this(0.01, 0, 0, 0);
    }

    /**
     * Creates a new PID function using specific PIDF variables
     * @param p proportional
     * @param i integral
     * @param d derivative
     * @param f feed-forward
     */
    public PID(double p, double i, double d, double f){
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
    }

    public void setP (double p){this.p = p;}


    /**
     * Gets a power level for motors to move towards target based on reference and state, and constructed PIDF variables
     * @param reference represents target position to be optimized towards
     * @param state represents the current position that should be optimized toward reference
     * @return double representing calculated power level or equivalent needed to move towards reference (directional)
     */
    public double getPowerLvl(double reference, double state){
        double error = reference - state;
        integralSum += (error * timer.seconds());
        double derivative = (error-lastError)/timer.seconds();

        lastError = error;
        timer.reset();

        return(
            (p * error      ) + 
            (i * integralSum) +
            (d * derivative ) +  
            (f * reference  )
        );
    }

    /**
     * Gets a power level for motors to move towards target based on error from target and constructed PIDF variables
     * @param error represents the distance or error between the target and current (AKA target - current or reference - state)
     * @return double representing calculated power level or equivalent needed to move towards reference (directional)
     */
    private double getPowerLvl(double error){
        integralSum += (error * timer.seconds());
        double derivative = (error-lastError)/timer.seconds();

        lastError = error;
        timer.reset();

        return(
                (p * error      ) +
                (i * integralSum) +
                (d * derivative ) +
                (f * error      )
        );
    }

}
