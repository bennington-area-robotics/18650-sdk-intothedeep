package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Collector;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Config
@Autonomous(group = "drive", name="AutonomousSampleDelivery")
public class AutonomousSampleDelivery extends AutoTemplate {

    //TODO FOR EBEN - clean up this code! remove the unnecessary code if its commented, implement the methods I added here

    public static double blueStartX = 24;
    public static double blueStartY = 63;
    public static double blueStartAng = 90;

    public static double redStartX = 0;
    public static double redStartY = -63;
    public static double redStartAng = -90;

    private static ElapsedTime runtime = new ElapsedTime();

    public static double basketX = 52, basketY = 49, basketHeading = 45;
    public static double ascentX = 5;
    public static double secondSampleX = 55, secondSampleY = 40;

    @Override
    public void runOpMode() throws InterruptedException {
        setBlueStartPose(blueStartX, blueStartY, blueStartAng);
        super.initialize();
        setManualCaching();
        while(!isStarted() && !isStopRequested()){
            collector.wristToHalfway();
            collector.closeGrip();
            tickAll();
        }
        waitForStart();


        run();
    }
    public void deliverToHighBasket(){
        performWithManualCaching(()-> arm.moveToTargetAngleBlocking(92, this::tickAll));
        arm.setTargetExtension(Arm.MAX_ARM_EXTENSION - 0.25);
        arm.setTargetAngle(92);
        while(Math.abs(arm.getExtension() - arm.getTargetExtension()) > 0.5){
            setManualCaching();
            tickAll();
        }
        setAutoCaching();
        //performWithManualCaching(()-> arm.moveToTargetExtensionBlocking(Arm.MAX_ARM_EXTENSION - 0.25, this::tickAll));
        performWithManualCaching(() -> arm.moveToTargetAngleBlocking(95, this::tickAll));
        performWithManualCaching(()-> collector.moveWristToBlocking(Collector.UP_POSITION, this::tickAll, false));
        waitTick(0.25);
        collector.openGrip();
        waitTick(0.75);
        performWithManualCaching(()-> collector.moveWristToBlocking(Collector.DOWN_POSITION, this::tickAll, false));
    }

    public void placePreloadedSample(){
        Trajectory moveToBasket = drive.trajectoryBuilder(getBlueStartPose(), true)
                .splineToSplineHeading(new Pose2d(40, 45, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(basketX, basketY, Math.toRadians(basketHeading)), Math.toRadians(basketHeading))
                .build();


        drive.followTrajectoryAsync(moveToBasket);
        collector.wristTo(0);

        while(drive.isBusy() && opModeIsActive()){

            collector.closeGrip();
            collector.wristToDefaultPosition();
            setAutoCaching();
            drive.update();
            setManualCaching();
            tickAll();
        }
        deliverToHighBasket();

    }
    public void collectSecondSample(){
        Trajectory moveToSample = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                .splineToConstantHeading(new Vector2d(secondSampleX, secondSampleY), Math.toRadians(-90))
                .build();

        performWithManualCaching(()->arm.moveToTargetExtensionBlocking(0, this::tickAll));
        performWithManualCaching(()->collector.moveWristToBlocking(90, this::tickAll, true));
        performWithManualCaching(()-> arm.moveToTargetAngleBlocking(0, this::tickAll));


        drive.followTrajectoryAsync(moveToSample);
        collector.wristTo(80);
        arm.setTargetAngle(0);
        arm.setTargetExtension(0);

        while(drive.isBusy() && opModeIsActive()){
            collector.wristToRotatedPosition();
            collector.openGrip();
            setAutoCaching();
            drive.update();
            setManualCaching();
            tickAll();
        }
        collector.closeGrip();
        waitTick(0.5);
        performWithManualCaching(()->collector.moveWristToBlocking(90, this::tickAll, true));
        collector.wristToDefaultPosition();

    }
    public void deliverSecondSample(){
        Trajectory moveToBasket = drive.trajectoryBuilder(drive.getPoseEstimate())
            .splineToSplineHeading(new Pose2d(basketX-1, basketY-1, Math.toRadians(basketHeading)), Math.toRadians(basketHeading))
            .build();

        setAutoCaching();
        drive.followTrajectoryAsync(moveToBasket);
        collector.wristTo(90);
        while(drive.isBusy()&&opModeIsActive()){
            collector.closeGrip();
            collector.wristToDefaultPosition();
            setAutoCaching();
            drive.update();
            setManualCaching();
            tickAll();
        }
        deliverToHighBasket();

    }

    public void pushSample(){
        Trajectory pushSample = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                .splineToSplineHeading(
                        new Pose2d(38, 45, Math.toRadians(90)),
                        Math.toRadians(-90),
                        velocityConstraint,
                        accelerationConstraint)
                .splineToSplineHeading(
                        new Pose2d(38,20, Math.toRadians(90)),
                        Math.toRadians(-90),
                        velocityConstraint,
                        accelerationConstraint)
                .splineToConstantHeading(new Vector2d(45,12), Math.toRadians(90), velocityConstraint, accelerationConstraint)
                .splineToConstantHeading(new Vector2d(45, 40), Math.toRadians(90),velocityConstraint, accelerationConstraint)
                .splineTo(new Vector2d(54, 59), Math.toRadians(45), velocityConstraint, accelerationConstraint)
                .build();

        performWithManualCaching(()->arm.moveToTargetExtensionBlocking(15, this::tickAll));
        arm.setTargetAngle(45);
        arm.setTargetExtension(0);
        drive.followTrajectoryAsync(pushSample);
        while(drive.isBusy() && opModeIsActive()){
            setAutoCaching();
            drive.update();
            setManualCaching();
            tickAll();
        }

    }

    public void moveToAscent(){
        Trajectory moveToAscent = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                .splineToSplineHeading(new Pose2d(48, 24, Math.toRadians(225)), Math.toRadians(225), velocityConstraint, accelerationConstraint)
                .splineToSplineHeading(new Pose2d(30, 10, Math.toRadians(180)), Math.toRadians(180), velocityConstraint, accelerationConstraint)
                .build();
        Trajectory touchRung = drive.trajectoryBuilder(moveToAscent.end())
                .forward(ascentX)
                .build();

        setAutoCaching();
        drive.followTrajectory(moveToAscent);
        performWithManualCaching(() -> arm.moveToTargetAngleBlocking(100, this::tickAll));
        setAutoCaching();
        drive.followTrajectory(touchRung);
        arm.setTargetAngle(100);
        collector.wristTo(215);
        setManualCaching();
        waitTick(1.5);
    }


    public void backUp(double distance){
        Trajectory backUp = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(distance)
                .build();
        setAutoCaching();
        drive.followTrajectory(backUp);
    }

    @Override
    public void run(){


        if (isStopRequested()) return;
        runtime.reset();
        collector.closeGrip();

        placePreloadedSample();
        //collectSecondSample();
        //deliverSecondSample();
        pushSample();
        moveToAscent();

        setManualCaching();
        arm.setTargetExtension(0);
        collector.wristTo(215);
        while(runtime.seconds() < 30 && opModeIsActive()){
            tickAll();
        }
    }
}