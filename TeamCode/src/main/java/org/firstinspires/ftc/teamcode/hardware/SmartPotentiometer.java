package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.utilities.PersistentStorage;

public class SmartPotentiometer {
	private final AnalogInput input;
	private final double maxAngle, maxVoltage, voltsPerDegree;
	private final String storageKey;

	private double offsetToZero;

	SmartPotentiometer(AnalogInput input, String name, double maxAngle, double maxVoltage){
		this.input = input;
		this.maxAngle = maxAngle;
		this.maxVoltage = maxVoltage;
		this.voltsPerDegree = maxVoltage / maxAngle;
		this.storageKey = "potentiometer_angle_offset(" + name + ")";
		this.offsetToZero = PersistentStorage.getDouble(storageKey, 0);
	}

	public double getAngle(){
		return normalizeDegrees(getRawAngle() + offsetToZero);
	}

	public double getRawAngle(){
		return input.getVoltage() / voltsPerDegree;
	}

	public void reset(){
		offsetToZero = -getRawAngle();
		PersistentStorage.saveDouble(storageKey, offsetToZero);
	}

	private double normalizeDegrees(double angle){
		if(angle > 0 && angle <= 360){
			return angle;
		}else if(angle < 0){
			return normalizeDegrees(angle + 360);
		}else {
			return angle % 360;
		}
	}
}
