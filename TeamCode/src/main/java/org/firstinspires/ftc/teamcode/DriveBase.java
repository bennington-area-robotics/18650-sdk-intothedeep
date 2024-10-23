package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;



public class DriveBase {
    //todo this class needs some work, rewrite this from scratch

    private final DcMotor
        LFront,
        RFront,
        LRear,
        RRear
    ;

    public DriveBase(HardwareMap hardwareMap){
        LFront = hardwareMap.get(DcMotor.class, "LFront");
        LRear = hardwareMap.get(DcMotor.class, "LRear");
        RFront = hardwareMap.get(DcMotor.class, "RFront");
        RRear = hardwareMap.get(DcMotor.class, "RRear");

        LFront.setDirection(DcMotor.Direction.REVERSE);
        LRear.setDirection(DcMotorSimple.Direction.FORWARD);
        RFront.setDirection(DcMotorSimple.Direction.FORWARD);
        RRear.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void reset(){
        DcMotor[] motors = {
            LFront,
            RFront,
            LRear,
            RRear
        };

        for(DcMotor motor : motors){
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    
    public void setPower(double rf, double lf, double rr, double lr){
        RFront.setPower(rf);
        LFront.setPower(lf);
        RRear.setPower(rr);
        LRear.setPower(lr);
    }
    public void setPower(double p){
        RFront.setPower(p);
        LFront.setPower(p);
        RRear.setPower(p);
        LRear.setPower(p);
    }
    public void setTargets(int rf, int lf, int rr, int lr){
        RFront.setTargetPosition(rf);
        LFront.setTargetPosition(lf);
        RRear.setTargetPosition(rr);
        LRear.setTargetPosition(lr);
    }
    
    public int getPosition(){return RFront.getCurrentPosition();}
    public double getPower(){return RFront.getPower();}
    public void stopMotors(){
        RFront.setPower(0);
        LFront.setPower(0);
        RRear.setPower(0);
        LRear.setPower(0);
    }


//    /**
//     * only Carl knows what this does. he's been gone since 2020.
//     * since we only used the right joystick 2023-2024, this only ever returned 1.
//     */
//    private double Get_Denominator(float lStickX, float lStickY, float rStickX) {
//        double sum = Math.abs(lStickY)
//                + Math.abs(lStickX)
//                + Math.abs(rStickX);
//        if (sum > 1) {
//            return sum;
//        } else {
//            return 1;
//        }
//    }

    /**
     * Sets the power values for the different motors.
     * To go straight/backwards, all LeftY values need to be equivalent.
     * To go sideways, the LeftX values of diagonally opposite motors need to be equivalent.
     * To rotate, the RightX values of motors on the same L/R side of the robot need to be equivalent.
     */
    public void setPowerValues(double LeftX, double LeftY, float RightX, double Power_Mod)
    {
        double lFront = (((-LeftY + LeftX) + RightX)/* / denominator*/) * Power_Mod;
        double lRear = (((-LeftY - LeftX) + RightX) /*/ denominator*/) * Power_Mod;
        double rFront = (((-LeftY - LeftX) - RightX) /*/ denominator*/) * Power_Mod;
        double rRear = (((-LeftY + LeftX) - RightX) /*/ denominator*/) * Power_Mod;

        LFront.setPower(lFront * 1.0007155);
        LRear.setPower(lRear * 1.0017707);
        RFront.setPower(rFront);
        RRear.setPower(rRear * 1.0032978);

    }

    public void GoStraight(float rightStickX, double powerLvl) {
        setPowerValues(
                0,
                -1 * powerLvl,
                rightStickX,
                powerLvl
        );
    }
    public void GoBackwards(float rightStickX, double powerLvl) {
        setPowerValues(
                0,
                powerLvl,
                rightStickX,
                powerLvl
        );
    }
    public void GoLeft(float rightStickX, double powerLvl) {
        setPowerValues(
                -1 * powerLvl,
                0,
                rightStickX,
                powerLvl
        );
    }
    public void GoRight(float rightStickX, double powerLvl) {
        setPowerValues(
                powerLvl,
                0,
                rightStickX,
                powerLvl
        );
    }



}
