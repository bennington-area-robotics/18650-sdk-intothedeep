package org.firstinspires.ftc.teamcode.hardware.controllers;

public class Consts {
    public double p, i, d, f, maxI;

    /**
     * Creates a full PIDF controller.
     *
     * @param p    the proportional coefficient. This controls how much the magnitude of the error affects the output.
     * @param i    the integral coefficient. This controls how much the overall change in error affects the output.
     * @param d    the derivative coefficient. This controls how much the change in the error affects the output.
     * @param f    the feed-forward constant. This is a directly outputted constant which is added to the output.
     * @param maxI the maximum of the integral sum. This controls the maximum amount the integral calculation can affect the output.
     */
    public static Consts of(double p, double i, double d, double f, double maxI){
        Consts constants = new Consts();
        constants.p = p;
        constants.i = i;
        constants.d = d;
        constants.f = f;
        constants.maxI = maxI;
        return constants;
    }

    /**
     * Creates a simple PF controller.
     *
     * @param p the proportional coefficient. This controls how much the magnitude of the error affects the output.
     * @param f the feed-forward constant. This is a directly outputted constant which is added to the output.
     */
    public static Consts of(double p, double f){
        return of(p, 0, 0, f, 0);
    }

    private Consts(){}

    public void set(double p, double i, double d, double f, double maxI) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
        this.maxI = maxI;
    }
}
