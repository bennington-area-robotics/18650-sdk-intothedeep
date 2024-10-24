package org.firstinspires.ftc.teamcode.hardware;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveBase {
    public final DriveBaseMotor leftFront, leftRear, rightFront, rightRear;
    private final DriveBaseMotor[] motors;

    public DriveBase(HardwareMap hardwareMap){
        this.leftFront = (DriveBaseMotor) hardwareMap.get(DcMotor.class, "LFront");
        this.leftRear = (DriveBaseMotor) hardwareMap.get(DcMotor.class, "LRear");
        this.rightFront = (DriveBaseMotor) hardwareMap.get(DcMotor.class, "RFront");
        this.rightRear = (DriveBaseMotor) hardwareMap.get(DcMotor.class, "RRear");

        motors = new DriveBaseMotor[]{
                leftFront,
                leftRear,
                rightFront,
                rightRear
        };
    }

    public void turnLeft(double angleDegrees){

    }

    public void turnRight(double angleDegrees){

    }

    public void forward(double distanceCentimeters){

    }

    public void backward(double distanceCentimeters){

    }

    public void left(double distanceCentimeters){

    }

    public void right(double distanceCentimeters){

    }

    public void setPower(double power){
        setPower(power, power, power, power);
    }
    public void setPower(double leftFront, double leftRear, double rightFront, double rightRear){
        this.leftFront.setPower(leftFront);
        this.leftRear.setPower(leftRear);
        this.rightFront.setPower(rightFront);
        this.rightRear.setPower(rightRear);
    }
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

    public void controllerInput(double x, double y, double angle){
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the correct ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(angle), 1);
        double leftFront = (y + x + angle) / denominator;
        double leftRear = (y - x + angle) / denominator;
        double rightFront = (y - x - angle) / denominator;
        double rightRear = (y + x - angle) / denominator;

        setPower(leftFront, leftRear, rightFront, rightRear);
    }
}
