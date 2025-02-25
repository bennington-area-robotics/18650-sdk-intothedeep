package org.firstinspires.ftc.teamcode.hardware.controllers;

public class BidirectionalPID extends PID {
    public Constants forwardSet;
    public Constants reverseSet;

    /**
     * Creates a PIDF controller which can be used to easily apply a control loop to almost anything.
     *
     * @param tolerance the distance at which the controller considers having reached the target.
     */
    public BidirectionalPID(Constants forwardSet, Constants reverseSet, double tolerance) {
        super(Constants.of(0, 0), tolerance);
        this.forwardSet = forwardSet;
        this.reverseSet = reverseSet;
    }

    @Override
    public double calc(double error) {
        if(error < 0)
            return calc(error, reverseSet);
        else {
            return calc(error, forwardSet);
        }
    }
}
