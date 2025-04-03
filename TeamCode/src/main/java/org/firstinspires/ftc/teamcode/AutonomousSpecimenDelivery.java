package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Config
@Autonomous(group = "drive", name="AutonomousSpecimenDeliveryPath")
public class AutonomousSpecimenDelivery extends AutoTemplate {

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
    public static double collectionArmAngle = 35, collectionArmExtension = 8, collectionCollectorAngle = -15;
    public static float collectionCollectorRotation = 0.4f;
    public static double deliveryCollectorPos = 65, deliveryArmAngle = 50, deliveryArmExtension =9;
    public static double forwardAmount = 32;
    public static double strafeAmount = 10.5;
    public static double offSet = 1.2;
    public static double secondCollectionY = 34.5;
    public static double secondSpecimenExtension = 9;

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

    public void placePreloadedSpecimen(){
        Trajectory moveToRung = drive.trajectoryBuilder(getBlueStartPose(), true)
                .back(forwardAmount,velocityConstraint,accelerationConstraint)
                //.splineToConstantHeading(new Vector2d(specX, specY), Math.toRadians(specHeading), velocityConstraint, accelerationConstraint)
                .build();
        performWithManualCaching(() -> arm.moveToTargetAngleBlocking(deliveryArmAngle, this::tickArm));
        performWithManualCaching(() -> collector.moveWristToBlocking(deliveryCollectorPos, this::tickAll, true));

        drive.followTrajectoryAsync(moveToRung);
        collector.wristToDefaultPosition();
        arm.setTargetExtension(deliveryArmExtension);

        while(drive.isBusy() && opModeIsActive()){

            collector.closeGrip();
            setAutoCaching();
            drive.update();
            setManualCaching();
            tickAll();
        }
        collector.openGrip();
        //wait(0.5);

    }

    public void collectSecondSpecimen(){
        Trajectory moveToCollectionPosition = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
            .splineToSplineHeading(new Pose2d(collectionX, collectionY, Math.toRadians(collectionHeading)), Math.toRadians(collectionTan))
            //.splineToConstantHeading(new Vector2d(collectionX, collectionY + strafeAmount), Math.toRadians(collectionHeading))
            .build();

        Trajectory strafeToPosition = drive.trajectoryBuilder(moveToCollectionPosition.end())
            .strafeLeft(strafeAmount)
            .build();

        //performWithManualCaching(() -> arm.moveToTargetExtensionBlocking(12, this::tickAll));
        //performWithManualCaching(() -> arm.moveToTargetAngleBlocking(collectionArmAngle, this::tickAll));

        drive.followTrajectoryAsync(moveToCollectionPosition);
        collector.rotateWristTo(collectionCollectorRotation);
        arm.setTargetAngle(collectionArmAngle);
        arm.setTargetExtension(collectionArmExtension);
        collector.wristTo(collectionCollectorAngle);

        while(drive.isBusy() && opModeIsActive()) {
            collector.openGrip();
            collector.rotateWristTo(collectionCollectorRotation);
            setAutoCaching();
            drive.update();
            setManualCaching();
            tickAll();
        }
        collector.wristTo(collectionCollectorAngle);
        arm.setTargetExtension(collectionArmExtension);
        drive.followTrajectoryAsync(strafeToPosition);
        while(drive.isBusy() && opModeIsActive()) {
            collector.rotateWristTo(collectionCollectorRotation);
            collector.openGrip();
            setAutoCaching();
            drive.update();
            setManualCaching();
            tickAll();
        }
        collector.closeGrip();

    }
    public void deliverSecondSpecimen(){
        Trajectory moveToRung = drive.trajectoryBuilder(drive.getPoseEstimate())
            .splineToConstantHeading(new Vector2d(-24, 55), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(-5, 45, Math.toRadians(90)), Math.toRadians(-90))
            .splineToConstantHeading(new Vector2d(-5, secondCollectionY), Math.toRadians(90), velocityConstraint, accelerationConstraint)
            .build();

        performWithManualCaching(() -> arm.moveToTargetAngleBlocking(deliveryArmAngle, this::tickAll));
        collector.wristToDefaultPosition();
        drive.followTrajectoryAsync(moveToRung);
        arm.setTargetExtension(secondSpecimenExtension);
        collector.wristTo(deliveryCollectorPos);

        while(drive.isBusy() && opModeIsActive()) {

            collector.closeGrip();
            collector.wristToDefaultPosition();
            setAutoCaching();
            drive.update();
            setManualCaching();
            tickAll();
        }
        collector.openGrip();

    }

    public void backUp(double distance){
        Trajectory backUp = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(distance)
                .build();
        setAutoCaching();
        drive.followTrajectory(backUp);
    }
    public void pushSample(){
        Trajectory pushSample = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                //.forward(10)
                .splineTo(
                        new Vector2d(-37, 45),
                        Math.toRadians(-90),
                        velocityConstraint,
                        accelerationConstraint)
                .splineTo(
                        new Vector2d(-37,20),
                        Math.toRadians(-90),
                        velocityConstraint,
                        accelerationConstraint)
                .splineToConstantHeading(
                        new Vector2d(-45,12),
                        Math.toRadians(90),
                        velocityConstraint,
                        accelerationConstraint)
                .splineTo(
                        new Vector2d(-45, 60),
                        Math.toRadians(90),
                        velocityConstraint,
                        accelerationConstraint)
                .build();

        arm.setTargetExtension(0);
        arm.setTargetAngle(0);
        collector.wristTo(215);
        drive.followTrajectoryAsync(pushSample);
        while(drive.isBusy() && opModeIsActive()) {

            collector.wristToDefaultPosition();
            setAutoCaching();
            drive.update();
            setManualCaching();
            tickAll();
        }
    }

    @Override
    public void run(){


        if (isStopRequested()) return;
        runtime.reset();
        collector.closeGrip();

        placePreloadedSpecimen();
        backUp(15);
        collectSecondSpecimen();
        waitTick(0.5);
        deliverSecondSpecimen();
        backUp(15);
        pushSample();

        setManualCaching();
        performWithManualCaching(()-> collector.moveWristToBlocking(220, this::tickAll, true));
        arm.setTargetAngle(0);
        //resetPosition();
        while(arm.getAngle() > 0){
            tickAll();
        }
        arm.setTargetExtension(0);
        //collector.wristTo(0);
        while(runtime.seconds() < 30 && opModeIsActive()){
            tickAll();
        }
    }
}