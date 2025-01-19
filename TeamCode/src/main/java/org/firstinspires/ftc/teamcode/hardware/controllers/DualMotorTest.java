package org.firstinspires.ftc.teamcode.hardware.controllers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Encoder;


@Config
@TeleOp(name="DualMotorAndroidStudio", group ="Into The Deep")
public class DualMotorTest extends LinearOpMode {

    public DcMotor leftMotor;
    public DcMotor rightMotor;
    public Encoder encoder;
    public ElapsedTime elapsedTime = new ElapsedTime();
    public double currentPos, prevPos, targetPos;
    public double vel;
    public double ticksPerDegree = 8192.0/360.0;

    // PID constants
    public static double kP = 0.0003;
    public static double kI = 0.0002;
    public static double kD = 0.0000001;

    private double lastError = 0;
    private double integralSum = 0;

    private FtcDashboard dashboard = FtcDashboard.getInstance();


    private int encoderOffset = 0;


    /**
     * Resets encoder on initialization
     */
    private void resetEncoder() {
        encoderOffset = encoder.getCurrentPosition();
        currentPos = 0;
        targetPos = 0;
        lastError = 0;
        integralSum = 0;
    }

    /**
     * Encoder keeps position after ending of the program, so this accounts for that fact
     * @return the position of the encoder based on the current program run
     *
     */
    private double getAdjustedPosition() {
        return (encoder.getCurrentPosition() - encoderOffset) / ticksPerDegree;
    }

    private double calculatePID(double currentPosition, double targetPosition) {
        double error = targetPosition - currentPosition;
        integralSum += error * elapsedTime.seconds();
        double derivative = (error - lastError) / elapsedTime.seconds();
        lastError = error;

        double output = (error * kP) + (integralSum * kI) + (derivative * kD);

        // Clamp final output
        return Math.min(Math.max(output, -1), 1);
    }


    @Override
    public void runOpMode() throws InterruptedException {

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        encoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightMotor"));

        currentPos = prevPos = targetPos = 0;
        vel = 0;

        //when motor power = 0, the motor will exert greater resistive force
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        encoder.setDirection(Encoder.Direction.REVERSE);

        //neither motor is using their built in encoder
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        if (opModeIsActive()) {
            elapsedTime.reset();
            resetEncoder();
            while (opModeIsActive()) {
                currentPos = getAdjustedPosition();

                //calculates velocity by measuring change in degrees over the interval
                if (elapsedTime.seconds() >= 0.1) {
                    vel = (currentPos - prevPos) / elapsedTime.seconds();
                    prevPos = currentPos;
                    elapsedTime.reset();
                }

                if (gamepad1.dpad_up) {
                    targetPos = currentPos + 90;
                    sleep(200);
                } else if (gamepad1.dpad_down) {
                    targetPos = currentPos - 90;
                    sleep(200);
                }else if (gamepad1.x) {
                    targetPos = 360;
                    sleep(200);
                } else if (gamepad1.y) {
                    targetPos = currentPos;
                }

                double posError = targetPos - currentPos;
                double motorPower = calculatePID(currentPos, targetPos);

                leftMotor.setPower(motorPower);
                rightMotor.setPower(motorPower);

                //0.5 degree tolerance
                if (Math.abs(posError) < 0.5) {
                    targetPos = currentPos;

                }

                telemetry.addData("left power", leftMotor.getPower());
                telemetry.addData("right power", rightMotor.getPower());
                telemetry.addData("Encoder Position", currentPos);
                telemetry.addData("Target Position", targetPos);
                telemetry.addData("Velocity (ticks/sec)", vel);
                telemetry.addData("Position Error", posError);
                telemetry.update();
            }
        }
    }
}

