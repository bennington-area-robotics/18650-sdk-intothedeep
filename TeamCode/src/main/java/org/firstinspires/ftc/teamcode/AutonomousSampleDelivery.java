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

    public static double blueStartX = 0;
    public static double blueStartY = 63;
    public static double blueStartAng = 90;

    public static double redStartX = 0;
    public static double redStartY = -63;
    public static double redStartAng = -90;

    private static ElapsedTime runtime = new ElapsedTime();

    public static double specX = 0, specY = 40, specTan = 90, specHeading = 90;
    public static double collectionX = -24, collectionY = 55, collectionTan = 180, collectionHeading = 0;
    public static double collectionArmAngle = 35, collectionArmExtension = 4, collectionCollectorAngle = -20;
    public static float collectionCollectorRotation = 0.4f;
    public static double deliveryCollectorPos = 40, deliveryArmAngle = 45, deliveryArmExtension = 23;
    public static double forwardAmount = 24.5;
    public static double strafeAmount = 10.5;
    public static double offSet = 4;
    public static double secondCollectionY = 40;

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

    public void placePreloadedSample(){
        Trajectory moveToBasket = drive.trajectoryBuilder(getBlueStartPose(), true)
                .splineToSplineHeading(new Pose2d(40, 57, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(57, 57, Math.toRadians(45)), Math.toRadians(45))
                .build();


        drive.followTrajectoryAsync(moveToBasket);
        collector.wristTo(0);

        while(drive.isBusy() && opModeIsActive()){

            collector.closeGrip();
            setAutoCaching();
            drive.update();
            setManualCaching();
            tickAll();
        }
        performWithManualCaching(()-> arm.moveToTargetAngleBlocking(100, this::tickAll));
        performWithManualCaching(()-> arm.moveToTargetExtensionBlocking(Arm.MAX_ARM_EXTENSION - 0.25, this::tickAll));
        performWithManualCaching(()-> collector.moveWristToBlocking(Collector.UP_POSITION, this::tickAll, false));
        collector.openGrip();
        wait(0.25);
        performWithManualCaching(()-> collector.moveWristToBlocking(Collector.DOWN_POSITION, this::tickAll, false));
        //wait(0.5);

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
                .splineTo(new Vector2d(52, 57), Math.toRadians(45), velocityConstraint, accelerationConstraint)
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
                .splineToSplineHeading(new Pose2d(21, 10, Math.toRadians(180)), Math.toRadians(180), velocityConstraint, accelerationConstraint)
                .build();
        drive.followTrajectoryAsync(moveToAscent);
        arm.setTargetAngle(100);
        collector.wristTo(0);
        while(drive.isBusy()){
            setAutoCaching();
            drive.update();
            setManualCaching();
            tickAll();
        }
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
        pushSample();
        moveToAscent();

        setManualCaching();
        arm.setTargetExtension(0);
        collector.wristTo(0);
        while(runtime.seconds() < 30 && opModeIsActive()){
            tickAll();
        }
    }
}