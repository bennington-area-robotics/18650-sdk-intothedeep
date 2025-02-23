package org.firstinspires.ftc.teamcode.hardware.controllers;

public class BidirectionalPID implements ControlAlg{
    public Constants forwardSet;
    public Constants reverseSet;
    private final PID pid;

    /**
     * Creates a PIDF controller which can be used to easily apply a control loop to almost anything.
     *
     * @param tolerance the distance at which the controller considers having reached the target.
     */
    public BidirectionalPID(Constants forwardSet, Constants reverseSet, double tolerance) {
        this.pid = new PID(Constants.of(0, 0), tolerance);
    }

    public double calc(double error) {
        if(error < 0)
            return pid.calc(error, reverseSet);
        else {
            return pid.calc(error, forwardSet);
        }
    }
}
