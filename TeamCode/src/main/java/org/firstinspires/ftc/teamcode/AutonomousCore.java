package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.hardware.drive.ConfiguredMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.drive.DriveConstants;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Config
@Autonomous(group = "drive")
public class AutonomousCore extends LinearOpMode {

    public static double startX = 0;
    public static double startY = 63;
    public static double startAng = 90;
    @Override
    public void runOpMode() throws InterruptedException {
        ConfiguredMecanumDrive drive = new ConfiguredMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(startX, startY, Math.toRadians(startAng));
        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;
        Trajectory moveToSamples = drive.trajectoryBuilder(startPose, true)
                //.splineTo(new Vector2d(-36, 48), Math.toRadians(-90))
                //.splineToSplineHeading(new Pose2d(-36,12, Math.toRadians(90)), Math.toRadians(-90))
                .splineTo(new Vector2d(-6, 40), Math.toRadians(-90))
                .splineTo(new Vector2d(-36, 48), Math.toRadians(180))
                .splineTo(new Vector2d(-36,12), Math.toRadians(-90))
                .build();

        /*Trajectory moveToSamples2 = drive.trajectoryBuilder(moveToSamples.end(),true)
                .lineToLinearHeading(new Pose2d(-36, 12,Math.toRadians(90)))
                .build();*/
        Trajectory moveToSamples3 = drive.trajectoryBuilder(startPose, true)
                .splineTo(new Vector2d(0, 45), Math.toRadians(-90), ConfiguredMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), ConfiguredMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-38, 48), Math.toRadians(-90))
                //.splineToSplineHeading(new Pose2d(-36, 48, Math.toRadians(90)), Math.toRadians(0))
                .splineTo(new Vector2d(-38,20), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-48,12), Math.toRadians(90))

                .splineToConstantHeading(new Vector2d(-48, 45), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-48, 55), Math.toRadians(-90), ConfiguredMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), ConfiguredMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //.splineToConstantHeading(new Vector2d(-48, 20), Math.toRadians(-90))
                //.splineToConstantHeading(new Vector2d(-59, 12), Math.toRadians(90))

//                .splineToConstantHeading(new Vector2d(-59, 45), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(-59, 55), Math.toRadians(-90), ConfiguredMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), ConfiguredMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(-59, 20), Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(-68, 12), Math.toRadians(90))
//
//                .splineToConstantHeading(new Vector2d(-68, 55), Math.toRadians(-90))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(moveToSamples3.end(), true)
                .splineToConstantHeading(new Vector2d(-48, 20), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-59, 12), Math.toRadians(90))

                .splineToConstantHeading(new Vector2d(-59, 45), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-59, 55), Math.toRadians(-90), ConfiguredMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), ConfiguredMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), true)
                .splineToConstantHeading(new Vector2d(-59, 20), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-68, 12), Math.toRadians(90))

                .splineToConstantHeading(new Vector2d(-68, 55), Math.toRadians(-90))
                .build();
        drive.followTrajectory(moveToSamples3);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);



        sleep(10000);
    }
}
