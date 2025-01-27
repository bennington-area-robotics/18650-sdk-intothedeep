package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Collector;
import org.firstinspires.ftc.teamcode.hardware.drive.DriveBase;

public class Autopilot {
    //todo these will be different per-side (these are for blue)
    private static final Pose2d basketPose = new Pose2d( 2.5 * 24, 2.5 * 24, Math.toRadians(0));
    private static final Pose2d submersiblePose = new Pose2d(0, 24, Math.toRadians(0));


    private final DriveBase driveBase;
    private final Arm arm;
    private final Collector collector;
    private Runnable tickRunnable;

    private Stage lastStage;

    public Autopilot(DriveBase driveBase, Arm arm, Collector collector){
        this.driveBase = driveBase;
        this.arm = arm;
        this.collector = collector;
    }

    public void setTickRunnable(Runnable runnable){
        tickRunnable = runnable;
    }

    public void start(){
        lastStage = findCurrentStage();
        run();
    }

    private void run(){

    }

    public Stage findCurrentStage(){
        if(holdingSample()){
            if(!armRetracted() && armHorizontal() && nearSubmersible()){
                return Stage.FINALIZE_COLLECTION;
            }else{
                if(approachingBasket()){
                    if(armReadyToDeliver() && atBasket()){
                        return Stage.READY_TO_DELIVER;
                    }else {
                        return Stage.PREPARE_DELIVERY;
                    }
                }else {
                    return Stage.MOVE_TO_BASKET;
                }
            }
        }else {
            if (nearSubmersible()) {
                if ((!armReadyToCollect() && collector.isWristTargetUp())) {
                    return Stage.PREPARE_TO_COLLECT;
                }else {
                    return Stage.AWAIT_USER_INTERVENTION;
                }
            }else if(!armReadyToCollect()){
                return Stage.PREPARE_FOR_RETURN_TO_SUBMERSIBLE;
            }else {
                return Stage.MOVE_TO_SUBMERSIBLE;
            }

        }
    }

    public enum Stage {
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
         * The robot is ready to deliver a sample to the basket. Allow user to finalize delivery.
         */
        READY_TO_DELIVER,
        /**
         * Preparing the robot to return to the submersible after a sample has been delivered to the basket.
         */
        PREPARE_FOR_RETURN_TO_SUBMERSIBLE,
        /**
         *  Moving the robot to the submersible after delivery.
         */
        MOVE_TO_SUBMERSIBLE,
        /**
         * Prepare the robot so the user can collect a peace easily.
         */
        PREPARE_TO_COLLECT
    }

    private boolean armReadyToDeliver(){
        return armVertical() && armExtended();
    }

    private boolean armReadyToCollect(){
        return armRetracted() && armHorizontal();
    }

    private boolean armExtended(){
        return errorTolerable(arm.getCachedExtension(), 38, 3);
    }

    private boolean armRetracted(){
        return errorTolerable(arm.getCachedExtension(), 0, 3);
    }

    private boolean armVertical(){
        return errorTolerable(arm.getCachedAngle(), 90, 5);
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
