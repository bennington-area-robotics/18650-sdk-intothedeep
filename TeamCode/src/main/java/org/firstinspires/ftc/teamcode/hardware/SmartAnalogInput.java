package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.hardware.filters.DataFilter;
import org.firstinspires.ftc.teamcode.hardware.filters.RollingAverage;

/**
 * This class is a wrapper for AnalogInputs that uses a rolling average to filter the direct voltage output for better consistency.
 */
public class SmartAnalogInput extends Device {

	private final AnalogInput base;
	private final DataFilter dataFilter;

	protected SmartAnalogInput(AnalogInput base, String configName){
		super(configName);
		this.base = base;
		this.dataFilter = new RollingAverage(100);
	}

	protected SmartAnalogInput(AnalogInput base, String configName, DataFilter dataFilter){
		super(configName);
		this.base = base;
		this.dataFilter = dataFilter;
	}

	public double getVoltage(){
		return dataFilter.compute(getRawVoltage());
	}

	public double getRawVoltage(){
		return base.getVoltage();
	}

}
