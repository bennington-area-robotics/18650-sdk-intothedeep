package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.SmartEncoder;
import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.controllers.GravityPID;
import org.firstinspires.ftc.teamcode.utilities.Direction;

@Config
public class PitchWrist {

	//config
	public static float WRIST_TICKS_PER_DEGREE = 8192f/360f;
	public static int UP_POSITION = 90, DOWN_POSITION = 0;
	public static double upKP = 0.005, upKI = 0.0000, upKD = 0.03, upKF = 0.1;
	public static double downKP = 0.005, downKI = 0.000, downKD = 0.0, downKF = -0.05;
	public static double kG = 0.3;

	private final SmartEncoder wristEncoder;

	private final GravityPID pid;

	public int targetAngle;

	private final SmartMotor motor;
	private Mode mode;
	private final TiltBase tiltBase;
	private double wristPower = 0.0;

	public enum Mode {
		MOVE_TO_TARGET, STAY_PARALLEL, STAY_PERPENDICULAR, FLOAT, SET_POWER
	}

	public PitchWrist(TiltBase tiltBase, SmartMotor wristMotor){
		this.motor = wristMotor;
		this.tiltBase = tiltBase;
		this.wristEncoder = motor.getEncoder();

		pid = new GravityPID.Builder()
				.forwardKP(() -> upKP)
				.forwardKI(() -> upKI)
				.forwardKD(() -> upKD)
				.forwardKF(() -> upKF)

				.reverseKP(() -> downKP)
				.reverseKI(() -> downKI)
				.reverseKD(() -> downKD)
				.reverseKF(() -> downKF)
				.g(() -> kG)
				.setGravityFunction((target, actual) -> Math.sin(Math.toRadians(tiltBase.getAngle() + getAngle())))
				.tolerance(1)
				.build();

		pid.setDirection(Direction.FORWARD);
		wristEncoder.reset();
		wristEncoder.setDirection(Direction.FORWARD);
		motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		setMode(Mode.FLOAT);
	}

	public void setWristPower(double wristPower) {
		this.wristPower = wristPower;
	}

	public double getWristPower() {
		return wristPower;
	}

	//wrist

	public void pitchTo(int position){
		targetAngle = position;
	}

	public void pitchUp(){
		pitchTo(UP_POSITION);
	}

	public void pitchDown(){
		pitchTo(DOWN_POSITION);
	}

	public boolean isUp(){
		return Helper.errorTolerable(getAngle(), UP_POSITION, 5);
	}

	public boolean isDown(){
		return Helper.errorTolerable(getAngle(), DOWN_POSITION, 5);
	}

	public boolean isTargetUp(){
		return Helper.errorTolerable(targetAngle, UP_POSITION, 5);
	}

	public boolean isTargetDown(){
		return Helper.errorTolerable(targetAngle, DOWN_POSITION, 5);
	}

	public double getAngle(){
		return (wristEncoder.getPosition() / WRIST_TICKS_PER_DEGREE);
	}

	public Mode getMode(){
		return mode;
	}

	public void setMode(Mode mode){
		this.mode = mode;
		if(this.mode == Mode.FLOAT){
			motor.setPower(0);
			motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		}
	}

	public double getVelocity(){
		return wristEncoder.getVelocity() / WRIST_TICKS_PER_DEGREE;
	}

	public int getTargetAngle() {
		return targetAngle;
	}

	/**
	 * Attempts to toggle the wrist target between up and down. If the target is not up or down, the target will not be changed.
	 * @return whether the wrist target was changed.
	 */
	public boolean toggle(){
		if (isTargetUp()) {
			pitchDown();
			return true;
		}else if (isTargetDown()) {
			pitchUp();
			return true;
		}else{
			return false;
		}
	}

	public void tick(){
		switch (mode) {
			case FLOAT:
				motor.setPower(0);
				break;
			case SET_POWER:
				motor.setPower(wristPower);
				break;
			case STAY_PARALLEL:
				motor.setPower(pid.calc(90 - tiltBase.getAngle(), getAngle()));
				break;
			case STAY_PERPENDICULAR:
				motor.setPower(pid.calc(tiltBase.getAngle() - 90, getAngle()));
				break;
			case MOVE_TO_TARGET:
				motor.setPower(pid.calc(targetAngle, getAngle()));
				break;
		}
	}

	private static class Helper {
		private static boolean errorTolerable(Number number1, Number number2, Number tolerance){
			return Math.abs(number2.doubleValue() - number1.doubleValue()) <= tolerance.doubleValue();
		}
	}
}