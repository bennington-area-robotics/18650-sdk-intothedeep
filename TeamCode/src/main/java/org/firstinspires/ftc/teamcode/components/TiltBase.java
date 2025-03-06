package org.firstinspires.ftc.teamcode.components;

import androidx.annotation.FloatRange;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.SmartEncoder;
import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.SmartTouchSensor;
import org.firstinspires.ftc.teamcode.hardware.controllers.GravityPID;
import org.firstinspires.ftc.teamcode.utilities.Await;

public class TiltBase {
	//<editor-fold desc="Config">
	public static float ARM_TICKS_PER_DEGREE = 65f; //this is a good estimate as of 1/24/2025

	public static double angleKP = 0.025, angleKI = 0, angleKD = 0.15, angleKF = 0.325, angleKG = 0;

	private final GravityPID pid;
	//</editor-fold>

	private final SmartMotor angleMotorRight;
	private final SmartMotor angleMotorLeft;

	private final SmartEncoder angleEncoder;
	public final SmartTouchSensor limitSensor;

	private final TelescopingArm telescopingArm;

	/**
	 * Target angle of the arm in degrees relative to the base. 0 is horizontal, while 90 is vertical.
	 */
	private double targetAngle = 0;

	private double tickOffsetToZero;

	public TiltBase(TelescopingArm telescopingArm, SmartMotor angleMotorLeft, SmartMotor angleMotorRight, SmartTouchSensor limitSensor) {
		//<editor-fold desc="Hardware Config">
		this.angleMotorRight = angleMotorRight;
		this.angleMotorLeft = angleMotorLeft;
		this.limitSensor = limitSensor;
		this.angleEncoder = angleMotorRight.getEncoder();
		this.telescopingArm = telescopingArm;

		this.angleMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		this.angleMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

		this.angleMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		this.angleMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		this.angleMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		this.angleMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		//</editor-fold>

		resetAngle();

		targetAngle = getAngle();

		pid = new GravityPID.Builder()
				.setGravityFunction((g, actual) -> g * Math.sin(Math.toRadians(getAngle())) * (19 + telescopingArm.getExtension()))

				.p(() -> angleKP)
				.i(() -> angleKI)
				.d(() -> angleKD)
				.f(() -> angleKF)
				.g(() -> angleKG)

				.tolerance(0.75)
				.build();
	}

	/**
	 * Get the current angle of the arm. This is relative to the base, at 0 the arm is horizontal, and at 90 the arm is vertical.
	 * Uses should be able to handle angles past 90 degrees, since the motor will not always land at exactly 90.
	 *
	 * @return the angle of the arm relative to the base.
	 */
	public double getAngle(){
		return (angleEncoder.getPosition() - tickOffsetToZero) / ARM_TICKS_PER_DEGREE;
	}

	public double getTargetAngle() {
		return targetAngle;
	}

	/**
	 * Checks if the passed target angle in inches is valid, then sets the target extension if so.
	 * A target angle is valid if the arm will not extend past the horizontal extension limit when at that target angle, and the current target extension.
	 *
	 * @param degrees the target angle in degrees.
	 * @return whether the operation was successful (whether it passed the checks).
	 */
	public boolean setTargetAngle(@FloatRange(from=0, to=100) double degrees){
		if(isValidAngle(degrees)){
			targetAngle = degrees;
			return true;
		}

		return false;
	}

	/**
	 * Sets the current angle as the zero position.
	 */
	public void resetAngle(){
		tickOffsetToZero = angleEncoder.getPosition();
	}

	/**
	 * Runs a controller cycle for the arm.
	 * This method should be called once per OpMode cycle to maintain the arm's position when at target,
	 * or adjust the arm's position when not at target. This controls both extension and retraction.
	 */
	public void tick(){
		if(limitSensor.isPressed())
			resetAngle();

		tickPIDF();
	}

	public double getPower(){
		return pid.result();
	}

	public boolean isBusy(){
		return pid.result() != 0;
	}

	/**
	 * Runs a cycle on the PIDF control loop for the arm.
	 */
	private void tickPIDF(){
		if(pid.result() != 0 && pid.calc(targetAngle, getAngle()) == 0)
			Await.notifyChange();

		angleMotorRight.setPower(pid.result());
		angleMotorLeft.setPower(pid.result());
	}

	public boolean isValidAngle(double degrees){
		return !(degrees < 0) && !(degrees > 100);
	}
}
