package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Collector;
import org.firstinspires.ftc.teamcode.hardware.drive.DriveBase;

public class Autopilot {
    private final DriveBase driveBase;
    private final Arm arm;
    private final Collector collector;
    private boolean isRunning;
    private Runnable tickRunnable;

    public Autopilot(DriveBase driveBase, Arm arm, Collector collector){
        this.driveBase = driveBase;
        this.arm = arm;
        this.collector = collector;
    }

    public void setTickRunnable(Runnable runnable){
        tickRunnable = runnable;
    }

    public void stop(){
        isRunning = false;
    }

    public void start(){
        isRunning = true;
    }

    public Stage findCurrentStage(){
        /*
            Arm extended, holding piece and at basket – await confirmation
            Holding piece and near basket – preparing for delivery
            Holding piece and arm extended horizontal – finalizing collection
            Holding piece and arm retracted- moving to basket
            No piece held and arm extended, arm not horizontal, not near submersible – prepare for movement
            No piece held and not at submersible- moving to submersible
            No piece held and at submersible with arm extended
     */
//        if(arm.getCachedExtension()){
//
//        }
        return Stage.DELIVER_SAMPLE;
    }

    public enum Stage{
        AWAIT_USER_INTERVENTION,
        MOVE_TO_BASKET, PREPARE_DELIVERY,
        AWAIT_USER_DELIVERY_CONFIRMATION, DELIVER_SAMPLE,
        LOWER_ARM, RETURN_TO_SUBMERSIBLE,
    }

    /*
    Arm extended, holding piece and at basket – await confirmation
    Holding piece and near basket – preparing for delivery
    Holding piece and arm extended horizontal – finalizing collection
    Holding piece and arm retracted- moving to basket
    No piece held and arm extended, arm not horizontal, not near submersible – prepare for movement
    No piece held and not at submersible- moving to submersible
    No piece held and at submersible with arm extended
     */

    private boolean errorTolerable(double number1, double number2, double tolerance){
        return Math.abs(number2 - number1) <= tolerance;
    }
}
