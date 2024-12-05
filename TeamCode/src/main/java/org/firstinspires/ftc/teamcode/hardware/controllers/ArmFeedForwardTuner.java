package org.firstinspires.ftc.teamcode.hardware.controllers;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
public class ArmFeedForwardTuner extends OpMode{



    private ArmFeedforward feedforward;
    public static double kS = 0, kCos = 0, kV = 0, kA = 0;


    private PIDController controller;
    public static double p =0, i = 0, d = 0;

    private double armPos;
    private double prevPos;
    double prevVelocity;
    public static double velocity = 0.0;
    public static double acceleration = 0.0;
    public static double target = 0.0;

    ElapsedTime loopTimer = new ElapsedTime();
    private final double fixedInterval = 0.02;

    private final double ticks_in_degree = 206.0/87.0;
    private final double ticks_in_radians = ticks_in_degree * (180.0/Math.PI);

    private DcMotorEx arm_motor;

    @Override
    public void init(){
        feedforward = new ArmFeedforward(kS, kCos, kV, kA);
        controller = new PIDController(p, i, d);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor = hardwareMap.get(DcMotorEx.class, "arm_motor");
        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armPos = prevPos = posInRadians();
        prevVelocity = 0;


        telemetry.addData("pos ", armPos);
        telemetry.addData("target", target);
        telemetry.update();


    }
    public double posInRadians(){
        return arm_motor.getCurrentPosition()/ ticks_in_radians;
    }

    @Override
    public void loop(){
        controller.setPID(p, i, d);
        feedforward = new ArmFeedforward(kS, kCos, kV, kA);
        if (loopTimer.seconds() >= fixedInterval) {
            armPos = posInRadians();

            velocity = (armPos - prevPos) / fixedInterval;
            acceleration = (velocity - prevVelocity)/ fixedInterval;

            double ff = feedforward.calculate(target/ticks_in_radians, velocity, acceleration);

            double pid = controller.calculate(arm_motor.getCurrentPosition(), target);
            double power = pid+ff;

            arm_motor.setPower(power);

            prevPos = armPos;
            prevVelocity = velocity;

            telemetry.addData("pos", arm_motor.getCurrentPosition());
            telemetry.addData("pos (radians)", armPos);
            telemetry.addData("target (radians)", target);
            telemetry.addData("power", power);
            telemetry.addData("time", loopTimer);
            telemetry.update();
            loopTimer.reset();
        }




    }
}
