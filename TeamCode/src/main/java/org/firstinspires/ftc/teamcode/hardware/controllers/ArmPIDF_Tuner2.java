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
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class ArmPIDF_Tuner2 extends OpMode{


    private PIDFController controller;
    private ArmFeedforward feedforward;
    public static double p =0, i = 0, d = 0;
    public static double f = 0;
    public static double kS = 0, kCos = 0, kV = 0, kA = 0;

    public static int target = 0;

    private final double ticks_in_degree = 86.0/28.0 * 280/360;
    private DcMotorEx arm_motor;

    @Override
    public void init(){
        controller = new PIDFController(p, i, d, f);
        feedforward = new ArmFeedforward(kS, kCos, kV, kA);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor = hardwareMap.get(DcMotorEx.class, "arm_motor");
        telemetry.addData("pos", arm_motor.getCurrentPosition());
        telemetry.addData("target", target);
        telemetry.update();

    }

    @Override
    public void loop(){
        while (true) {
            controller.setPIDF(p, i, d, f);
            int armPos = arm_motor.getCurrentPosition();
            double output = controller.calculate(armPos, target);
            arm_motor.setVelocity(output);
            telemetry.addData("pos", armPos);
            telemetry.addData("target", target);
            telemetry.update();
        }

    }
}
