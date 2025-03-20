package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.ArmController;
import org.firstinspires.ftc.teamcode.components.Grip;
import org.firstinspires.ftc.teamcode.components.PitchWrist;
import org.firstinspires.ftc.teamcode.components.RollWrist;
import org.firstinspires.ftc.teamcode.components.TelescopingArm;
import org.firstinspires.ftc.teamcode.components.TiltBase;
import org.firstinspires.ftc.teamcode.vision.MultiAprilTagReader;
import org.firstinspires.ftc.teamcode.components.DriveBase;
import org.firstinspires.ftc.teamcode.utilities.Pose;
import org.firstinspires.ftc.teamcode.hardware.Hardware;

import java.util.List;

@Config
public abstract class OpModeCore extends BasicOpModeCore {

	//<editor-fold desc="Fields">
	//components
	protected static MultiAprilTagReader aprilTagReader;
	protected static OpModeCore instance;
	protected static Grip grip;
	protected static PitchWrist pitch;
	protected static RollWrist roll;
	protected static DriveBase driveBase;
	protected static TelescopingArm telescoping;
	protected static TiltBase tilt;
	protected static ArmController arm;
	protected List<LynxModule> lynxModules;
	protected ElapsedTime tickTimer;
	//</editor-fold>

	//<editor-fold desc="Instance Getters">
	public static OpModeCore getInstance(){
		return instance;
	}
	//</editor-fold>

	@Override
	protected void initialize(){
		super.initialize();
		instance = this;

		//initialize hardware
		driveBase = new DriveBase(hardwareMap);

		telescoping = new TelescopingArm(
				Hardware.getMotor("extensionMotor"),
				Hardware.getTouchSensor("extensionLimitSensor")
		);
		tilt = new TiltBase(
				telescoping,
				Hardware.getMotor("tiltMotorLeft"),
				Hardware.getMotor("tiltMotorRight", true),
				Hardware.getTouchSensor("tiltLimitSensor")
		);


		grip = new Grip(
				Hardware.getServo("gripServo")
		);
		pitch = new PitchWrist(
				tilt,
				Hardware.getMotor("wristMotor", true)
		);
		roll = new RollWrist(
				Hardware.getServo("wristServo")
		);


		aprilTagReader = new MultiAprilTagReader(
				Hardware.getCamera(
						"Webcam Left",
						new Pose(-6.5, 2.125, 90)
				),
				Hardware.getCamera(
						"Webcam Right",
						new Pose(6.5, 2.125, -90)
				)
		);

		arm = new ArmController(telescoping, tilt);

		tickTimer = new ElapsedTime();

		// always configure telemetry last
		configureTelemetry();
	}

	protected void configureTelemetry(){

		prettyTelem.addLine("System Status")
				.addData("Tick Time", () -> Math.round(tickTimer.milliseconds()))
				.addData("Localization: ", () -> driveBase.getPoseSimple())
		;

		prettyTelem.addLine("Tilt")
				.addData("Current Angle", () -> tilt.getAngle())
				.addData("Target Angle", () -> tilt.getTargetAngle())
				.addData("Power", () -> tilt.getPower())
				.addData("Limit Sensor Pressed?", () -> tilt.limitSensor.isPressed());

		prettyTelem.addLine("Extension")
				.addData("Current Length", () -> telescoping.getExtension())
				.addData("Target Length", () -> telescoping.getTargetExtension())
				.addData("Power", () -> telescoping.getPower())
				.addData("Limit Sensor Pressed?", () -> telescoping.limitSensor.isPressed());

		prettyTelem.addLine("Grip")
				.addData("Position", () -> grip.getGripPosition())
				.addData("Open?", () -> grip.isGripOpen())
				.addData("Closed?", () -> grip.isGripClosed());

		prettyTelem.addLine("Pitch Wrist")
				.addData("Position", () -> pitch.getAngle())
				.addData("Target", pitch::getTargetAngle)
				.addData("Power", pitch::getWristPower)
				.addData("Up/Down", () -> pitch.isUp() ? "Up" : pitch.isDown() ? "Down" : "No");

		prettyTelem.addLine("April Tags")
				.addData("Left Camera", () -> aprilTagReader.getFirstPose(0).toString())
				.addData("Right Camera", () -> aprilTagReader.getFirstPose(1).toString());

	}

	public void tick(){
		super.tick();
		tilt.tick();
		telescoping.tick();
		pitch.tick();
		tickTimer.reset();
	}
}
