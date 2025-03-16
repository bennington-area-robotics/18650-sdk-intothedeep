package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.AnalogInput;

import java.util.ArrayDeque;

/**
 * This class is a wrapper for AnalogInputs that uses a rolling average to filter the direct voltage output for better consistency.
 */
public class SmartAnalogInput extends Device {

	private final AnalogInput base;
	private final RollingAverage rollingAverage;

	protected SmartAnalogInput(AnalogInput base, String configName){
		super(configName);
		this.base = base;
		this.rollingAverage = new RollingAverage(100);
	}

	public double getVoltage(){
		return rollingAverage.roll(getRawVoltage());
	}

	public double getRawVoltage(){
		return base.getVoltage();
	}

	public static class RollingAverage {
		private int maxSize;
		private final ArrayDeque<Double> values;
		private double sum = 0.0;

		public RollingAverage(int maxSize) {
			this.maxSize = maxSize;
			this.values = new ArrayDeque<>(maxSize);
		}

		/** @noinspection DataFlowIssue*/
		public void add(double value) {
			if (values.size() >= maxSize) {
				sum -= values.pollFirst(); // Remove oldest value
			}
			values.addLast(value);
			sum += value;
		}

		public double getAverage() {
			return values.isEmpty() ? 0.0 : sum / values.size();
		}

		public double roll(double newValue) {
			add(newValue);
			return getAverage();
		}

		public int size() {
			return values.size();
		}

		public void setMaxSize(int newMaxSize) {
			if (newMaxSize <= 0) throw new IllegalArgumentException("maxSize must be greater than 0");

			this.maxSize = newMaxSize;

			// Trim excess values if the new maxSize is smaller
			while (values.size() > maxSize) {
				//noinspection DataFlowIssue
				sum -= values.pollFirst(); // Remove oldest value to match new max size
			}
		}

		public int getMaxSize() {
			return maxSize;
		}

		@NonNull
		@Override
		public String toString() {
			return values.toString();
		}
	}
}
