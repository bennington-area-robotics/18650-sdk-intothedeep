package org.firstinspires.ftc.teamcode.core.implementations;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.BasicOpModeCore;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.SmartPotentiometer;
import org.firstinspires.ftc.teamcode.hardware.filters.RollingAverage;
import org.firstinspires.ftc.teamcode.hardware.filters.RollingMedian;

@Config
@TeleOp(name="PotentiometerAvgTest")
public class RollingAvgTest extends BasicOpModeCore {

	public static boolean enabled = true;

	SmartPotentiometer potentiometer;

	@Override
	protected void initialize(){
		super.initialize();
		potentiometer = Hardware.getPotentiometer("encoder", 360, 3.3, new RollingAverage(100));
		prettyTelem.addDataToDashboard("Voltage", () -> enabled ? potentiometer.getVoltage() : potentiometer.getRawAngle());

		prettyTelem.addDataToDashboard("Angle", potentiometer::getAngle);
	}

	@Override
	public void tick(){
		super.tick();
	}
}
