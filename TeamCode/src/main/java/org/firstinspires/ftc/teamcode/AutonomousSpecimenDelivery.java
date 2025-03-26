package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.apriltag.AprilTagReader;
import org.firstinspires.ftc.teamcode.apriltag.Camera;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Collector;
import org.firstinspires.ftc.teamcode.hardware.drive.ConfiguredMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.drive.DriveBase;
import org.firstinspires.ftc.teamcode.hardware.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.hardware.drive.Pose;

import java.util.List;

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

    public static double specX = 0, specY = 40, specTan = 90, specHeading = 90;
    public static TrajectoryVelocityConstraint velocityConstraint = ConfiguredMecanumDrive.getVelocityConstraint(
            30,
            2,
            DriveConstants.TRACK_WIDTH);

    public static TrajectoryAccelerationConstraint accelerationConstraint = ConfiguredMecanumDrive.getAccelerationConstraint(
            20);

    private static AutonomousCore instance;

    public static double extensionVar = 16.22;

    private final Pose2d blueStartPose = new Pose(blueStartX, blueStartY, blueStartAng).toRR();
    private final Pose2d redStartPose = new Pose(redStartX, redStartY, redStartAng).toRR();

    public static AutonomousCore getInstance() {
        return instance;
    }

    public static Telemetry getTelemetry() {
        return instance.telemetry;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        setStartPose(blueStartX, blueStartY, blueStartAng);
        super.initialize();

        waitForStart();

        run();
    }

    public void blueObservationSamplePushPath(){


        Trajectory moveRobot = drive.trajectoryBuilder(getBlueStartPose(), true)
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
                        new Vector2d(-45, 55),
                        Math.toRadians(90),
                        velocityConstraint,
                        accelerationConstraint)
                .build();

        Trajectory moveRobot2 = drive.trajectoryBuilder(moveRobot.end(), true)

                .splineTo(new Vector2d(-45, 20), Math.toRadians(-90), velocityConstraint, accelerationConstraint)
                .splineToConstantHeading(new Vector2d(-53, 12), Math.toRadians(90), velocityConstraint, accelerationConstraint)

                .splineTo(new Vector2d(-53, 55), Math.toRadians(90), velocityConstraint, accelerationConstraint)
                .build();

        Trajectory moveRobot3 = drive.trajectoryBuilder(moveRobot2.end(), true)
                .splineTo(new Vector2d(-53, 20), Math.toRadians(-90),velocityConstraint, accelerationConstraint)
                .splineToConstantHeading(new Vector2d(-61, 12), Math.toRadians(90), velocityConstraint, accelerationConstraint)
                .splineTo(new Vector2d(-62, 60), Math.toRadians(90), velocityConstraint, accelerationConstraint)
                .build();
        drive.followTrajectory(moveRobot);
        drive.followTrajectory(moveRobot2);
        drive.followTrajectory(moveRobot3);
    }
    public void placeSpecimen(){
        Trajectory moveToRung = drive.trajectoryBuilder(getBlueStartPose(), true)
                .splineToConstantHeading(new Vector2d(specX, specY), Math.toRadians(specHeading))
                .build();
        arm.setTargetAngle(43.5);
        drive.followTrajectoryAsync(moveToRung);
        collector.wristTo(40);
        while(drive.isBusy()){
            drive.update();
            tick();
        }

    }

    @Override
    public void run(){


        if (isStopRequested()) return;

        placeSpecimen();
        collector.moveWristToBlocking(0, this::tick, false);
    }

}