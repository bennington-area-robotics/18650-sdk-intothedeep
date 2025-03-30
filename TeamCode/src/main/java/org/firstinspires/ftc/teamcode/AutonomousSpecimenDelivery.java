package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
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
    public static double collectorPos = 40, armAnglePos = 45, armExtensionPos = 18.5;
    public static double forwardAmount = 25;

    @Override
    public void runOpMode() throws InterruptedException {
        setBlueStartPose(blueStartX, blueStartY, blueStartAng);
        super.initialize();

        waitForStart();

        run();
    }

    public void placeSpecimen(){
        Trajectory moveToRung = drive.trajectoryBuilder(getBlueStartPose(), true)
                .back(forwardAmount,velocityConstraint,accelerationConstraint)
                //.splineToConstantHeading(new Vector2d(specX, specY), Math.toRadians(specHeading), velocityConstraint, accelerationConstraint)
                .build();
        arm.moveToTargetAngleBlocking(armAnglePos, this::tickArm);
        collector.moveWristToBlocking(collectorPos, this::tickAll, true);
        drive.followTrajectoryAsync(moveToRung);
        collector.wristToDefaultPosition();
        arm.setTargetExtension(armExtensionPos);
        while(drive.isBusy() && opModeIsActive()){
            collector.closeGrip();
            drive.update();
            tickAll();
        }
        collector.openGrip();
        double currentTime = runtime.seconds();
        while(runtime.seconds() < currentTime + 1){
            tickAll();
        }

    }
    public void backUp(){
        Trajectory backUp = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(10)
                .build();
        drive.followTrajectory(backUp);
    }

    @Override
    public void run(){


        if (isStopRequested()) return;
        runtime.reset();
        collector.closeGrip();

        placeSpecimen();
        backUp();
        resetPosition();
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