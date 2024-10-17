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
    public void setTargets(int rf, int lf, int rr, int lr){
        RFront.setTargetPosition(rf);
        LFront.setTargetPosition(lf);
        RRear.setTargetPosition(rr);
        LRear.setTargetPosition(lr);
    }
    
    public int getPosition(){return RFront.getCurrentPosition();}
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

    /**
     * Moves the robot-base in a specified direction to a target position.
     * @param direction: direction that the drive base will move (1:forward, 2:back, 3:left, 4:right, 5:rotate left, 6:rotate right)
     * @param target: the amount of rotation ticks the motors will move
     * @param timeout: ignore this ngl, set it to 4 or something it's unimportant
     * @param targetSpeed: the speed the robot will ramp up to
     * @param ramp: the rate at which the robot ramps, set to 0 if you don't want to ramp
     */
    public void setMotorTargets(int direction, int target, int timeout, double targetSpeed, double ramp) {
        double 
            powerMin = 0.2,     // motor power at beginning and end of ramp up and down
            speed = powerMin,   // initial motor power
            increment = ramp,   // motor power increment
            distanceToReachTargetSpeed = 0
        ;

        double incrementInterval = 0.25; // interval between incrementing power (in seconds)
        boolean set = false; // makes distanceToReachTargetSpeed run only once

        DcMotor[] motors = {
            LFront, 
            LRear, 
            RFront, 
            RRear
        };

        switch(direction){
            case 1: // forward
                LFront.setTargetPosition(target);
                LRear.setTargetPosition(target);
                RFront.setTargetPosition(target);
                RRear.setTargetPosition(target);
                break;

            case 2: // back
                LFront.setTargetPosition(-target);
                LRear.setTargetPosition(-target);
                RFront.setTargetPosition(-target);
                RRear.setTargetPosition(-target);
                break;

            case 3: // left
                LFront.setTargetPosition(-target);
                LRear.setTargetPosition(target);
                RFront.setTargetPosition(target);
                RRear.setTargetPosition(-target);
                break;

            case 4: // right
                LFront.setTargetPosition(target);
                LRear.setTargetPosition(-target);
                RFront.setTargetPosition(-target);
                RRear.setTargetPosition(target);
                break;

            case 5: // rotate left
                LFront.setTargetPosition(-target);
                LRear.setTargetPosition(-target);
                RFront.setTargetPosition(target);
                RRear.setTargetPosition(target);
                break;

            case 6: // rotate right
                LFront.setTargetPosition(target);
                LRear.setTargetPosition(target);
                RFront.setTargetPosition(-target);
                RRear.setTargetPosition(-target);
                break;
        }

        for(DcMotor motor : motors){
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if(ramp < 0.01) {
                speed = targetSpeed;
                motor.setPower(speed);
            }
            motor.setPower(speed);
        }

        ElapsedTime runtime = new ElapsedTime(), rampTimer = new ElapsedTime();
        while(
            /*opModeIsActive()
            &&*/ (runtime.seconds() < timeout)
            && (LFront.isBusy() && LRear.isBusy() && RFront.isBusy() && RRear.isBusy())
        ){
            if (ramp > 0.00) {
                double averageMotorDistance = ( // creates a variable equal to the average distance between each motor's current position and its target position
                        Math.abs(LFront.getCurrentPosition() - LFront.getTargetPosition()) +
                                Math.abs(LRear.getCurrentPosition() - LRear.getTargetPosition()) +
                                Math.abs(RFront.getCurrentPosition() - RFront.getTargetPosition()) +
                                Math.abs(RRear.getCurrentPosition() - RRear.getTargetPosition())
                ) / 4.0;

                if (speed >= targetSpeed && !set) {
                    distanceToReachTargetSpeed = target - averageMotorDistance;
                    set = true;
                }

                if (Math.abs(averageMotorDistance) > target / 2.0 && speed < targetSpeed) { // if not half way through and not reached target speed then speed up
                    if (rampTimer.seconds() >= incrementInterval) {
                        speed += increment;
                        rampTimer.reset();
                    }
                } else if ( // if half way through or at the distance required to ramp down, and speed is greater than powerMin, then slow down
                        averageMotorDistance <= distanceToReachTargetSpeed
                                || averageMotorDistance > target / 2.0
                ) {
                    if (rampTimer.seconds() >= incrementInterval) {
                        if (speed >= powerMin) {
                            speed -= increment;
                            rampTimer.reset();
                        } else { // in case speed goes lower than powerMin then set it to powerMin
                            speed = powerMin;
                        }
                    }
                }

            }

            LFront.setPower(speed);
            LRear.setPower(speed);
            RFront.setPower(speed);
            RRear.setPower(speed);

        }
        reset();
    }


}
