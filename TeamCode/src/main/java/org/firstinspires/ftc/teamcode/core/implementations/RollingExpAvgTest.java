package org.firstinspires.ftc.teamcode.core.implementations;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.BasicOpModeCore;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.SmartPotentiometer;
import org.firstinspires.ftc.teamcode.hardware.filters.ExponentialRollingAverage;
import org.firstinspires.ftc.teamcode.hardware.filters.RollingMedian;

@Config
@TeleOp(name="PotentiometerExpAvgTest")
public class RollingExpAvgTest extends BasicOpModeCore {

	public static boolean enabled = true;

	SmartPotentiometer potentiometer;

	@Override
	protected void initialize(){
		super.initialize();
		potentiometer = Hardware.getPotentiometer("encoder", 360, 3.3, new ExponentialRollingAverage(0.05));
		prettyTelem.addDataToDashboard("Voltage", () -> enabled ? potentiometer.getVoltage() : potentiometer.getRawAngle());

		prettyTelem.addDataToDashboard("Angle", potentiometer::getAngle);
	}

	@Override
	public void tick(){
		super.tick();
	}
}
