package org.firstinspires.ftc.teamcode.hardware.controllers;


public class PIDController {
    private double kP, kI, kD;
    private double setPoint;  // target position
    private double previousError = 0;
    private double integral = 0;
    private double lastTime = 0;



    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
    public void reset(){
        lastTime = 0;
    }

    public void setSetPoint(double setPoint) {
        this.setPoint = setPoint;
    }
    public void setkP(double p){this.kP = p;}
    public void setkD(double d){this.kD = d;}
    public void setkI(double i){this.kI = i;}
    public void setK(double value, int index){
        switch(index){
            case 0: kP = value;
            break;
            case 1: kD = value;
            break;
            case 2: kI = value;
            break;
        }
    }

    public double calculate(double currentPosition, double currentTime) {

        double deltaTime = (currentTime - lastTime) / 1000.0;  // Convert ms to seconds

        if (lastTime == 0) {
            lastTime = currentTime;
            return 0;
        }

        // Calculate error
        double error = setPoint - currentPosition;

        // Proportional term
        double proportional = kP * error;

        // Integral term
        integral += error * deltaTime;
        double integralTerm = kI * integral;

        // Derivative term
        double derivative = (error - previousError) / deltaTime;
        double derivativeTerm = kD * derivative;

        // Update previous error and time
        previousError = error;
        lastTime = currentTime;

        // Total output (motor power)
        double output = proportional + integralTerm + derivativeTerm;


        return output;
    }
}

