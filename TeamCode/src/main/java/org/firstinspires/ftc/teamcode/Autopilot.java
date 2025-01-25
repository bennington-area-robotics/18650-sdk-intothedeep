package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Collector;
import org.firstinspires.ftc.teamcode.hardware.drive.DriveBase;

public class Autopilot {
    //todo give these real values
    private static final Pose2d basketPose = new Pose2d( 0, 0, Math.toRadians(0));
    private static final Pose2d submersiblePose = new Pose2d(0, 0, Math.toRadians(0));


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
        if(armReadyToDeliver() && atBasket() && holdingSample()){
            return Stage.READY_TO_DELIVER;
        } else if (holdingSample() && approachingBasket()) {
            return Stage.PREPARE_DELIVERY;
        } else if (holdingSample() && nearSubmersible() && !armRetracted() && armHorizontal()) {
            return Stage.FINALIZE_COLLECTION;
        } else if (holdingSample() && !atBasket()) {
            return Stage.MOVE_TO_BASKET;
        } else if (holdingSample()) { //todo edit this clause (placeholder condition)
            return Stage.PREPARE_FOR_RETURN_TO_SUBMERSIBLE;
        }else{ //todo add more clauses here
            return Stage.AWAIT_USER_INTERVENTION;
        }
    }

    public enum Stage{
        /**
         * Waiting for the driver to intervene, this is usually triggered when autopilot encounters unexpected or unhandled conditions.
         */
        AWAIT_USER_INTERVENTION,
        /**
         * Finalizing a collection and preparing the robot for movement to the basket.
         */
        FINALIZE_COLLECTION,
        /**
         * Moving the robot to the basket after collection.
         */
        MOVE_TO_BASKET,
        /**
         * Preparing the robot for delivering a sample to the basket.
         */
        PREPARE_DELIVERY,
        /**
         * The robot is ready to deliver a sample to the basket.
         */
        READY_TO_DELIVER,
        /**
         * Preparing the robot to return to the submersible after a sample has been delivered to the basket.
         */
        PREPARE_FOR_RETURN_TO_SUBMERSIBLE,
        /**
         *
         */
        RETURN_TO_SUBMERSIBLE,
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

    private boolean armReadyToDeliver(){
        return armVertical() && armExtended();
    }

    private boolean armExtended(){
        return errorTolerable(arm.getCachedExtension(), 38, 3);
    }

    private boolean armRetracted(){
        return errorTolerable(arm.getCachedExtension(), 0, 3);
    }

    private boolean armVertical(){
        return errorTolerable(arm.getCachedAngle(), 90, 3);
    }

    private boolean armHorizontal(){
        return errorTolerable(arm.getCachedAngle(), 0, 5);
    }

    private boolean atBasket(){
        return withinInches(driveBase.getPoseEstimate(), basketPose, 1);
    }

    private boolean approachingBasket(){
        return withinInches(driveBase.getPoseEstimate(), basketPose, 24);
    }

    private boolean nearSubmersible(){
        return withinInches(driveBase.getPoseEstimate(), submersiblePose, 2);
    }

    private boolean holdingSample(){
        return collector.holdingSample();
    }

    private boolean errorTolerable(double number1, double number2, double tolerance){
        return Math.abs(number2 - number1) <= tolerance;
    }

    private double getDistance(Pose2d pose1, Pose2d pose2){
        return Math.sqrt(Math.pow(pose2.getX() - pose1.getX(), 2) + Math.pow(pose2.getY() - pose1.getY(), 2));
    }

    private boolean withinInches(Pose2d pose1, Pose2d pose2, double distance){
        return getDistance(pose1, pose2) <= distance;
    }
}
