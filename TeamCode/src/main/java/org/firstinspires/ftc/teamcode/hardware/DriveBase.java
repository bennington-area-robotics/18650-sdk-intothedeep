package org.firstinspires.ftc.teamcode.hardware;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveBase {
    public final DcMotor leftFront, leftRear, rightFront, rightRear;
    private final DcMotor[] motors;
    private static double powerFactor = 1;

    public DriveBase(HardwareMap hardwareMap){
        this.leftFront = hardwareMap.get(DcMotor.class, "LFront");
        this.leftRear = hardwareMap.get(DcMotor.class, "LRear");
        this.rightFront = hardwareMap.get(DcMotor.class, "RFront");
        this.rightRear = hardwareMap.get(DcMotor.class, "RRear");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = new DcMotor[]{
                leftFront,
                leftRear,
                rightFront,
                rightRear
        };
    }

    /**
     * Moves the robot in the given direction, with the given power, while turning with the given turn power.
     * @param power the power to move at, this does not necessarily mean all motors will be given this power.
     * @param direction the angle at which to move. This does not turn the robot. 0 is forward, 90 is right, etc
     * @param turnPower the power at which to turn.
     */
    public void moveDirection(double power, double direction, double turnPower){
        double x = Math.sin(direction) * power;
        double y = Math.cos(direction) * power;

        move(x, y, turnPower);
    }

    /**
     * Moves the robot in the given direction, with the given power..
     * @param power the power to move at, this does not necessarily mean all motors will be given this power.
     * @param direction the angle at which to move. This does not turn the robot. 0 is forward, 90 is right, etc
     */
    public void moveDirection(double power, double direction){
        double x = Math.sin(direction) * power;
        double y = Math.cos(direction) * power;

        move(x, y, 0);
    }

    /**
     * Moves and turns the robot using general power modifiers in each direction/axis
     * @param x the power to move left/right with. Positive -> right, Negative -> left
     * @param y the power to move forward/back with. Positive -> forward, Negative -> backward
     * @param turn the power to turn with. Positive -> turn right, Negative -> turn left
     */
    public void move(double x, double y, double turn){
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the correct ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);
        double leftFront = (y + x + turn) / denominator;
        double leftRear = (y - x + turn) / denominator;
        double rightFront = (y - x - turn) / denominator;
        double rightRear = (y + x - turn) / denominator;

        setPower(leftFront, leftRear, rightFront, rightRear);
    }

    /**
     * Moves and turns the robot using general power modifiers in each direction/ axis
     * @param x the power to move left/ right with. Positive -> right, Negative -> left
     * @param y the power to move forward/ back with. Positive -> forward, Negative -> backward
     */
    public void move(double x, double y){
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the correct ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);
        double leftFront = (y + x) / denominator;
        double leftRear = (y - x) / denominator;
        double rightFront = (y - x) / denominator;
        double rightRear = (y + x) / denominator;

        setPower(leftFront * powerFactor, leftRear * powerFactor, rightFront * powerFactor, rightRear * powerFactor);
    }

    /**
     * Sets the power to all motors on the drive base.
     * @param power the power to set motors to. <b>0-1</b>
     */
    public void setPower(double power){
        setPower(power, power, power, power);
    }

    /**
     * @param leftFront power to left front motor.
     * @param leftRear power to left rear motor.
     * @param rightFront power to right front motor.
     * @param rightRear power to right rear motor.
     */
    public void setPower(double leftFront, double leftRear, double rightFront, double rightRear){
        this.leftFront.setPower(leftFront);
        this.leftRear.setPower(leftRear);
        this.rightFront.setPower(rightFront);
        this.rightRear.setPower(rightRear);
    }
    public void setPowerFactor(double factor){

        powerFactor = factor;
    }

    /**
     * Stops all motors. This is a shortcut method for <code>driveBase.setPower(0)</code>`.
     */
    public void stop(){
        setPower(0);
    }

    /**
     * Stops all drive motors, then resets their encoders. After resetting all drive power will be 0.
     */
    public void reset(){
        stop();
        for(DcMotor motor : motors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
