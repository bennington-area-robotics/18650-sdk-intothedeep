package org.firstinspires.ftc.teamcode.hardware.controllers;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Config
@Autonomous(group = "drive")
public class TrajectoryTester extends LinearOpMode {

    public static double startX = -11.5;
    public static double startY = 63;
    public static double startAng = -90;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(startX, startY, Math.toRadians(startAng));
        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory trajToBars = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(0, 28), 0)
                .build();

        Trajectory midTraj = drive.trajectoryBuilder(trajToBars.end(), true)
                .splineToConstantHeading(new Vector2d(-35.5, 48), 0  )
                .build();

        Trajectory midTraj2 = drive.trajectoryBuilder(midTraj.end())
                .splineTo(new Vector2d(-35.5, 0), Math.toRadians(180))
                .build();
        /*Trajectory midTraj3 = drive.trajectoryBuilder(midTraj2.end())
                .lineToSplineHeading(new Pose2d(30,0))
                .build();
*/

        drive.followTrajectory(trajToBars);
        drive.followTrajectory(midTraj);
        drive.followTrajectory(midTraj2);
        //drive.followTrajectory(midTraj3);
    }
}
