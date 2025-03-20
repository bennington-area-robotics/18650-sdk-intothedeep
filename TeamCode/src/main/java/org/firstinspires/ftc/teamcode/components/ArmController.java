package org.firstinspires.ftc.teamcode.components;

import org.firstinspires.ftc.teamcode.core.BasicOpModeCore;
import org.firstinspires.ftc.teamcode.utilities.Await;
import org.firstinspires.ftc.teamcode.utilities.ChainedFuture;
import org.firstinspires.ftc.teamcode.utilities.TaskScheduler;

public class ArmController {
	private final TelescopingArm telescoping;
	private final TiltBase tilt;
	private final TaskScheduler scheduler;

	public ArmController(TelescopingArm telescoping, TiltBase tilt){
		this.telescoping = telescoping;
		this.tilt = tilt;
		this.scheduler = new TaskScheduler(8);
	}

	public void tiltTo(double angle){
		tilt.setTargetAngle(angle);
		while(tilt.isBusy()){
			tilt.tick();
			BasicOpModeCore.getTelemetry().update();
		}
	}

	public ChainedFuture<?> tiltToAsync(double angle){
		return scheduler.runAsync(() -> {
			tilt.setTargetAngle(angle);
			Await.notification(tilt.noLongerBusyNotifier);
		});
	}

	public void telescopeTo(double extension){
		telescoping.setTargetExtension(extension);
		while(telescoping.isBusy()){
			telescoping.tick();
			BasicOpModeCore.getTelemetry().update();
		}
	}

	public ChainedFuture<?> telescopeToAsync(double extension){
		return scheduler.runAsync(() -> {
			telescoping.setTargetExtension(extension);
			Await.notification(telescoping.noLongerBusyNotifier);
		});
	}
}
