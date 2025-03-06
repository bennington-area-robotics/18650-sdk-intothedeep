package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.ArmController;
import org.firstinspires.ftc.teamcode.components.TelescopingArm;
import org.firstinspires.ftc.teamcode.components.TiltBase;
import org.firstinspires.ftc.teamcode.utilities.GameState;
import org.firstinspires.ftc.teamcode.utilities.PrettyTelemetry;
import org.firstinspires.ftc.teamcode.vision.MultiAprilTagReader;
import org.firstinspires.ftc.teamcode.components.Collector;
import org.firstinspires.ftc.teamcode.components.DriveBase;
import org.firstinspires.ftc.teamcode.utilities.Pose;
import org.firstinspires.ftc.teamcode.hardware.Hardware;

import java.util.List;
import java.util.Locale;

@Config
public abstract class OpModeCore extends BasicOpModeCore {

	//<editor-fold desc="Fields">
	//components
	protected static MultiAprilTagReader aprilTagReader;
	protected static OpModeCore instance;
	protected static Collector collector;
	protected static DriveBase driveBase;
	protected static TelescopingArm telescoping;
	protected static TiltBase tilt;
	protected static GameState gameState;
	protected static ArmController arm;
	protected List<LynxModule> lynxModules;
	protected ElapsedTime tickTimer;
	//</editor-fold>

	//<editor-fold desc="Instance Getters">
	public static OpModeCore getInstance(){
		return instance;
	}

	public static PrettyTelemetry getTelemetry(){
		return instance.prettyTelem;
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


		collector = new Collector(
				tilt,
				"colorSensor",
				"wristMotor",
				"gripServo"
		);

		gameState = new GameState(driveBase, tilt, telescoping, collector);

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

	@Override
	protected void configureTelemetry(){

		prettyTelem.addLine("System Status")
				.addData("Tick Time", () -> Math.round(tickTimer.milliseconds()))
				.addData("Stage", () -> gameState.findCurrentStage())
				.addData("Localization: ", () -> driveBase.getPoseSimple())
		;
		prettyTelem.addLine("Game State")
				.addData("In Basket Area", () -> gameState.inBasketArea())
				.addData("In Submersible Collection Area", () -> gameState.isInSubmersibleCollectionArea())
				.addData("In Observation Collection Area", () -> gameState.inObservationZoneCollectionArea())
				.addData("In Specimen Delivery Area", () -> gameState.inSpecimenDeliveryArea())
		;

		prettyTelem.addLine("Arm Status")
				.addData("Current Angle", () -> tilt.getAngle())
				.addData("Target Angle", () -> tilt.getTargetAngle())
				.addData("Current Extension", () -> telescoping.getExtension())
				.addData("Target Extension", () -> telescoping.getTargetExtension())
				.addData("Tilt Power", () -> tilt.getPower())
				.addData("Extension Power", () -> telescoping.getPower())
				.addData("Tilt Limit Sensor Pressed?", () -> tilt.limitSensor.isPressed())
				.addData("Extension Limit Sensor Pressed?", () -> telescoping.limitSensor.isPressed());

		prettyTelem.addLine("Grip")
				.addData("Position", () -> collector.getGripPosition())
				.addData("Open?", () -> collector.isGripOpen())
				.addData("Closed?", () -> collector.isGripClosed());

		prettyTelem.addLine("Wrist")
				.addData("Position", () -> collector.getWristAngle())
				.addData("Target", collector::getWristTarget)
				.addData("Up?", () -> collector.isWristUp())
				.addData("Down?", () -> collector.isWristDown());

		prettyTelem.addLine("Color Sensor")
				.addData("HSV", this::getHSV)
				.addData("RGB", this::getRGB)
				.addData("Scoring Color", () -> collector.colorSensor.getScoringElementColor());

		prettyTelem.addLine("April Tags")
				.addData("Left Camera", () -> aprilTagReader.getFirstPose(0).toString())
				.addData("Right Camera", () -> aprilTagReader.getFirstPose(1).toString());
	}

	protected String getHSV(){
		float[] hsv = collector.colorSensor.getHSV();
		return String.format(Locale.ENGLISH, "Hue: %.3f Saturation: %.3f Value: %.3f", hsv[0], hsv[1], hsv[2]);
	}

	protected String getRGB(){
		NormalizedRGBA rgba = collector.colorSensor.getNormalizedColors();
		return String.format(Locale.ENGLISH, "Red: %.3f Green: %.3f Blue: %.3f", rgba.red, rgba.green, rgba.blue);
	}

	@Override
	public void runOpMode(){
		initialize();
		waitForStart();
		while(opModeIsActive()){
			tick();
		}
	}

	public void tick(){
		super.tick();
		tilt.tick();
		telescoping.tick();
		collector.tick();
		tickTimer.reset();
	}
}
