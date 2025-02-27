package org.firstinspires.ftc.teamcode.hardware.controllers;

public class BidirectionalPID extends PID {
    public PIDFConstants forwardSet;
    public PIDFConstants reverseSet;

    /**
     * Creates a PIDF controller which can be used to easily apply a control loop to almost anything.
     *
     * @param tolerance the distance at which the controller considers having reached the target.
     */
    public BidirectionalPID(PIDFConstants forwardSet, PIDFConstants reverseSet, double tolerance) {
        super(PIDFConstants.of(0, 0), tolerance);
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
