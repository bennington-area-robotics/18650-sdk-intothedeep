package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.teamcode.utilities.PersistentStorage;

import java.util.ArrayDeque;

/**
 * A class that converts an analog signal into a potentiometer based absolute encoder with intelligent zeroing, caching, and persistent storage.
 * This class allows reading angular positions, adjusting offsets, and saving calibration values for future use.
 * It uses a rolling average to filter the direct voltage output for better consistency.
 */
public class SmartPotentiometer extends Device implements Caching{
	private final SmartAnalogInput input;
	private final double maxAngle, maxVoltage, voltsPerDegree;
	private final String storageKey;
	private double offsetToZero;
	private final HardwareCache<Double> rawAngleCache;

	/**
	 * Constructs a `SmartPotentiometer` instance with the given parameters.
	 *
	 * @param input      The `AnalogInput` representing the potentiometer's analog sensor.
	 * @param name       The unique name used for persistent storage of the offset value.
	 * @param maxAngle   The maximum angle (in degrees) the potentiometer can measure.
	 * @param maxVoltage The maximum voltage output of the potentiometer at its highest position.
	 */
	SmartPotentiometer(SmartAnalogInput input, String name, double maxAngle, double maxVoltage) {
		super(name);
		this.input = input;
		this.maxAngle = maxAngle;
		this.maxVoltage = maxVoltage;
		this.voltsPerDegree = maxVoltage / maxAngle;
		this.storageKey = "potentiometer_angle_offset(" + name + ")";
		this.offsetToZero = PersistentStorage.getDouble(storageKey, 0);
		this.rawAngleCache = new HardwareCache<>(() -> input.getVoltage() / voltsPerDegree);
	}

	/**
	 * Gets the current angle of the potentiometer, adjusted for the stored offset.
	 *
	 * @return The adjusted angular position in degrees.
	 */
	public double getAngle() {
		return normalizeAngle(getRawAngle() + offsetToZero, maxAngle);
	}

	/**
	 * Gets the raw angle of the potentiometer without applying any offset.
	 * This is calculated based on the sensor's voltage and the conversion factor.
	 *
	 * @return The raw angular position in degrees.
	 */
	public double getRawAngle() {
		return rawAngleCache.read();
	}

	/**
	 * Resets the potentiometer's zero position to the current raw angle.
	 * This effectively sets the current position as the new zero and saves the offset persistently.
	 */
	public void reset() {
		offsetToZero = -getRawAngle();
		PersistentStorage.saveDouble(storageKey, offsetToZero);
	}

	/**
	 * Removes any previous offset of the angle.
	 */
	public void clearOffset() {
		offsetToZero = 0;
		PersistentStorage.saveDouble(storageKey, offsetToZero);
	}

	/**
	 * Normalizes an angle to be within the range [0, maxAngle].
	 * Ensures that negative angles wrap around and large angles are reduced within bounds.
	 *
	 * @param angle    The angle to be normalized.
	 * @param maxAngle The maximum allowable angle before wrapping occurs.
	 * @return The normalized angle within the range [0, maxAngle). Returns in the units of angle & maxAngle.
	 */
	private static double normalizeAngle(double angle, double maxAngle) {
		angle %= maxAngle;

		if (angle < 0) {
			angle += maxAngle;
		}

		return angle == maxAngle ? 0 : angle;
	}

	public double getVoltage(){
		return input.getVoltage();
	}

	/**
	 *
	 */
	@Override
	public void invalidateCache(){
		rawAngleCache.invalidateCache();
	}

	/**
	 *
	 */
	@Override
	public void updateCache(){
		rawAngleCache.updateCache();
	}

	@Override
	public void setStrategy(Strategy strategy){
		rawAngleCache.setStrategy(strategy);
	}

	@Override
	public Strategy getStrategy(){
		return rawAngleCache.getStrategy();
	}
}
