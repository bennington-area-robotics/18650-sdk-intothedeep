package org.firstinspires.ftc.teamcode.hardware.controllers;

import java.util.function.DoubleSupplier;

public class GravityPID extends PID {
    private final GravityFunction gravityFunc;
    private final DoubleSupplier g;

    private GravityPID(
            DoubleSupplier kP, DoubleSupplier kI, DoubleSupplier kD, DoubleSupplier kF,
            GravityFunction gravityFunc, DoubleSupplier g, double tolerance
    ) {
        super(kP, kI, kD, kF, tolerance);
        this.gravityFunc = gravityFunc;
        this.g = g;
    }

    @Override
    public double calc(double target, double actual) {
        double basePID = super.calc(target, actual);
        double gravityEffect = gravityFunc.apply(g.getAsDouble(), actual) * g.getAsDouble();
        lastResult = basePID + gravityEffect;
        return lastResult;
    }

    public static class Builder {
        private DoubleSupplier kP = () -> 0, kI = () -> 0, kD = () -> 0, kF = () -> 0;
        private GravityFunction gravityFunc = (t, a) -> 0;
        private DoubleSupplier g = () -> 0;
        private double tolerance;

        public Builder p(double kP) {
            this.kP = () -> kP;
            return this;
        }
        public Builder p(DoubleSupplier kP) {
            this.kP = kP;
            return this;
        }
        public Builder i(double kI) {
            this.kI = () -> kI;
            return this;
        }
        public Builder i(DoubleSupplier kI) {
            this.kI = kI;
            return this;
        }
        public Builder d(double kD) {
            this.kD = () -> kD;
            return this;
        }
        public Builder d(DoubleSupplier kD) {
            this.kD = kD;
            return this;
        }
        public Builder f(double kF) {
            this.kF = () -> kF;
            return this;
        }
        public Builder f(DoubleSupplier kF) {
            this.kF = kF;
            return this;
        }
        public Builder setGravityFunction(GravityFunction func) {
            this.gravityFunc = func;
            return this;
        }
        public Builder g(double g) {
            this.g = () -> g;
            return this;
        }
        public Builder g(DoubleSupplier g) {
            this.g = g;
            return this;
        }
        public Builder tolerance(double tolerance){
            this.tolerance = tolerance;
            return this;
        }

        public GravityPID build() { return new GravityPID(kP, kI, kD, kF, gravityFunc, g, tolerance); }
    }


    @FunctionalInterface
    public interface GravityFunction {
        double apply(double kG, double actual);
    }
}