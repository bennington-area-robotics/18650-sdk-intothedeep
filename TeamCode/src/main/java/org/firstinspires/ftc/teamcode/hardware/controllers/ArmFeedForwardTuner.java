package org.firstinspires.ftc.teamcode.hardware.controllers;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
public class ArmFeedForwardTuner extends OpMode{



    private ArmFeedforward feedforward;
    public static double kS = 0, kCos = 0.002, kV = 0, kA = 0;

    private PIDController controller;
    public static double p =0.001, i = 0, d = 0.00005;

    private double armPos;
    private int armPosTicks;
    private double prevPos;
    double prevVelocity;
    public static double velocity = 0.0;
    public static double acceleration = 0.0;

    public static double target = 0.0;
    private static double targetChecker = 0.0;
    private static boolean isWaitingForMovement = true;

    private ElapsedTime loopTimer = new ElapsedTime();
    private final double fixedInterval = 0.02;

    private final double ticks_in_degree = 206.0/87.0;
    private final double ticks_in_radians = ticks_in_degree * (180.0/Math.PI);

    private DcMotorEx arm_motor;

    private TrapezoidalProfile motionProfile;
    private ElapsedTime profileTimer;

    @Override
    public void init(){

        feedforward = new ArmFeedforward(kS, kCos, kV, kA);
        controller = new PIDController(p, i, d);

        motionProfile = new TrapezoidalProfile(4.0, 2.0);  // maxVel = 2 rad/s, maxAccel = 1 rad/s^2
        profileTimer = new ElapsedTime();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor = hardwareMap.get(DcMotorEx.class, "arm_motor");
        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armPos = prevPos = 0.0;
        armPosTicks = 0;
        prevVelocity = 0;

        telemetry.update();
        profileTimer.reset();
        loopTimer.reset();


    }
    public double posInRadians(int armPos){
        return armPos / ticks_in_radians;
    }

    @Override
    public void loop(){

        //takes in live updates from FTC Dashboard
        controller.setPID(p, i, d);
        feedforward = new ArmFeedforward(kS, kCos, kV, kA);

        //Updates trapezoidal motion profile when the target changes
        if (Math.abs(targetChecker - target) > 1 ){
            profileTimer.reset();
            motionProfile.setNewTarget(
                    arm_motor.getCurrentPosition(),
                    target,
                    0.0
            );
            isWaitingForMovement = true;
        }

        double actualInterval = loopTimer.seconds();    
        if (actualInterval >= fixedInterval) {
            loopTimer.reset();

            armPosTicks = arm_motor.getCurrentPosition();
            armPos = posInRadians(armPosTicks);

            double maxVelocity = 2.0;

            velocity = (armPos - prevPos) / actualInterval;
            acceleration = (velocity - prevVelocity) / actualInterval;

            if (isWaitingForMovement && Math.abs(velocity) > 0.25) {  // Adjust threshold as needed
                profileTimer.reset();  // Now we start the timer
                isWaitingForMovement = false;
            }

            double profiledPosition;

            double profileTime = profileTimer.seconds();
            if (isWaitingForMovement) {
                // While waiting, use the starting position as the target
                profiledPosition = armPosTicks;
            } else {
                // Once moving, follow the profile
                profiledPosition = motionProfile.calculate(profileTime);
            }

            double ff = feedforward.calculate(profiledPosition/ticks_in_radians, velocity, acceleration);

            double pid = controller.calculate(armPosTicks, profiledPosition);
            double power = pid+ff;

            if (Math.abs(velocity) > maxVelocity) {
                power *= maxVelocity / Math.abs(velocity);
            }

            arm_motor.setPower(power);

            prevPos = armPos;
            prevVelocity = velocity;

            telemetry.addData("pos", armPosTicks);
            telemetry.addData("pos (radians)", armPos);
            telemetry.addData("target ", target);
            telemetry.addData("target (radians)", target/ticks_in_radians);
            telemetry.addData("velocity", velocity);
            telemetry.addData("actualInterval", actualInterval);
            telemetry.addData("time", loopTimer);
            telemetry.addData("Current Pos", armPosTicks);
            telemetry.addData("Profiled Target", profiledPosition);
            telemetry.addData("Profile Time", profileTime);
            telemetry.addData("Acceleration", acceleration);
            telemetry.addData("Raw FF Output", ff);
            telemetry.addData("Raw PID Output", pid);
            telemetry.addData("Final Power", power);
            telemetry.addData("waiting for movement", isWaitingForMovement);
            telemetry.update();

        }
        targetChecker = target;




    }
}
