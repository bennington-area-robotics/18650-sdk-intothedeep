package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.SmartTouchSensor;
import org.firstinspires.ftc.teamcode.hardware.controllers.DirectionalPID;
import org.firstinspires.ftc.teamcode.utilities.Await;

public class TelescopingArm {
	//<editor-fold desc="Config">

	public static float ARM_TICKS_PER_INCH = 190f;
	public static double MAX_ARM_EXTENSION = 37.0;

	public static double MAX_HORIZONTAL_EXTENSION = 38.0;

	public static double extensionKP = 0.2, extensionKI, extensionKD, extensionKF = 0.15;
	public static double retractionKP = 0.2, retractionKI, retractionKD, retractionKF = 0.45;

	private final DirectionalPID pid = new DirectionalPID.Builder()
			.forwardKP(() -> extensionKP)
			.forwardKI(() -> extensionKI)
			.forwardKD(() -> extensionKD)
			.forwardKF(() -> extensionKF)

			.reverseKP(() -> retractionKP)
			.reverseKI(() -> retractionKI)
			.reverseKD(() -> retractionKD)
			.reverseKF(() -> retractionKF)

			.tolerance(0.75)
			.build();
	//</editor-fold>

	private final SmartMotor spool;
	public final SmartTouchSensor limitSensor;

	/**
	 * Target extension of the arm in inches past the minimum extension (not extended at all)
	 */
	private double targetExtension = 0;

	public TelescopingArm(SmartMotor spoolMotor, SmartTouchSensor limitSensor) {
		this.spool = spoolMotor;
		this.limitSensor = limitSensor;

		this.spool.setDirection(DcMotorSimple.Direction.REVERSE);

		resetExtension();
		targetExtension = getExtension();
	}

	/**
	 * Get the current extension of the end of the arm past the minimum extension (fully retracted).
	 *
	 * @return the extension of the end of the arm.
	 */
	public double getExtension(){
		return spool.getCurrentPosition() / ARM_TICKS_PER_INCH;
	}

	public double getTargetExtension(){
		return targetExtension;
	}

	/**
	 * Checks if the passed target extension in inches is valid, then sets the target extension if so.
	 * A target extension is valid if the arm will not extend past the horizontal extension limit when at the current target angle, and that target extension.
	 *
	 * @param inches the target extension in inches.
	 * @return whether the operation was successful (whether it passed the checks).
	 */
	public boolean setTargetExtension(double inches){
		if(isValidExtension(inches)) {
			targetExtension = inches;
			return true;
		}

		return false;
	}

	/**
	 * Checks if the passed target extension in inches is valid, then sets the target extension if so.
	 * A target extension is valid if the arm will not extend past the horizontal extension limit when at the current target angle, and that target extension.
	 *
	 * @param inches the target extension in inches.
	 * @return whether the operation was successful (whether it passed the checks).
	 */
	private boolean setTargetExtensionIgnoreMacro(double inches){
		if(isValidExtension(inches)) {
			targetExtension = inches;
			return true;
		}

		return false;
	}

	/**
	 * Sets the current extension as the target and zero position.
	 */
	public void resetExtension(){
		spool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		targetExtension = 0;
		spool.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}

	/**
	 * Runs a controller cycle for the arm.
	 * This method should be called once per OpMode cycle to maintain the arm's position when at target,
	 * or adjust the arm's position when not at target. This controls both extension and retraction.
	 */
	public void tick(){
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
		if(pid.result() != 0 && pid.calc(targetExtension, getExtension()) == 0)
			Await.notifyChange();

		spool.setPower(pid.result());
	}

	public boolean isValidExtension(double inches){
		if(inches > MAX_ARM_EXTENSION || inches < 0)
			return false;

		targetExtension = inches;

		return true;
	}
}
