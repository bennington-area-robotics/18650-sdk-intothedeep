package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.utilities.PersistentStorage;
import org.firstinspires.ftc.teamcode.utilities.PrettyTelemetry;

public abstract class BasicOpModeCore extends LinearOpMode {
	protected PrettyTelemetry prettyTelem;

	@Override
	public void runOpMode(){
		initialize();
		waitForStart();
		while(opModeIsActive()){
			tick();
		}
	}

	protected void initialize(){
		Hardware.init(hardwareMap);
		PersistentStorage.init(hardwareMap);
		this.prettyTelem = new PrettyTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
		StandardTrackingWheelLocalizer.reverseEncoders();
	}

	protected void configureTelemetry(){

	}

	public void tick(){
		Hardware.invalidateCaches();
		prettyTelem.update();
	}
}
