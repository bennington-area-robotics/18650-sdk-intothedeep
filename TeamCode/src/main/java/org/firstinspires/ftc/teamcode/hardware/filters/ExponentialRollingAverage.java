package org.firstinspires.ftc.teamcode.hardware.filters;

public class ExponentialRollingAverage implements DataFilter {
	private final double alpha;
	private Double ema = null;

	public ExponentialRollingAverage(double alpha){
		this.alpha = alpha;
	}

	public double compute(double value){
		ema = (ema == null) ? value : (alpha * value + (1 - alpha) * ema);
		return ema;
	}
}
