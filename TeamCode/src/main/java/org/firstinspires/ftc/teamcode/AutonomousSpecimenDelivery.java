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
    public static double collectionArmAngle = 35, collectionArmExtension = 4, collectionCollectorAngle = -20;
    public static float collectionCollectorRotation = 0.4f;
    public static double deliveryCollectorPos = 40, deliveryArmAngle = 45, deliveryArmExtension = 23;
    public static double forwardAmount = 23;
    public static double strafeAmount = 9.5;
    public static double offSet = 4;
    public static double secondCollectionY = 40;

    @Override
    public void runOpMode() throws InterruptedException {
        setBlueStartPose(blueStartX, blueStartY, blueStartAng);
        super.initialize();

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
            .splineToConstantHeading(new Vector2d(collectionX, collectionY + strafeAmount), Math.toRadians(collectionHeading))
            .build();

        performWithManualCaching(() -> arm.moveToTargetAngleBlocking(collectionArmAngle, this::tickAll));
        performWithManualCaching(() -> arm.moveToTargetExtensionBlocking(10, this::tickAll));
        drive.followTrajectoryAsync(moveToCollectionPosition);
        collector.rotateWristTo(collectionCollectorRotation);
        //arm.setTargetAngle(collectionArmAngle);
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
        arm.setTargetExtension(deliveryArmExtension-offSet);
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

    public void backUp(){
        Trajectory backUp = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(3)
                .build();
        setAutoCaching();
        drive.followTrajectory(backUp);
    }

    @Override
    public void run(){


        if (isStopRequested()) return;
        runtime.reset();
        collector.closeGrip();

        placePreloadedSpecimen();
        backUp();
        collectSecondSpecimen();
        wait(0.5);
        deliverSecondSpecimen();
        backUp();
        performWithManualCaching(() -> arm.moveToTargetExtensionBlocking(0, this::tickAll));

        setManualCaching();
        arm.setTargetAngle(0);
        collector.wristUp();
        //resetPosition();
        while(arm.getAngle() > 0){
            tickAll();
        }
        arm.setTargetExtension(0);
        collector.wristTo(0);
        while(runtime.seconds() < 30 && opModeIsActive()){
            tickAll();
        }
    }
}