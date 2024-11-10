package org.firstinspires.ftc.teamcode.hardware.controllers;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class TrajectoryTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory trajToBars = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(30, 12.5), 0)
                .build();

        Trajectory midTraj = drive.trajectoryBuilder(trajToBars.end(), true)
                .splineToConstantHeading(new Vector2d(14, -11.5), 0  )
                .build();

        /*
        Trajectory midTraj2 = drive.trajectoryBuilder(midTraj.end())
                .splineTo(new Vector2d(15, -5.5), Math.toRadians(-135))
                .build();
        Trajectory midTraj3 = drive.trajectoryBuilder(midTraj2.end())
                .splineTo(new Vector2d(22, -4), Math.toRadians(-180))
                .build();

         */
        drive.followTrajectory(trajToBars);
        drive.followTrajectory(midTraj);
        //drive.followTrajectory(midTraj2);
        //drive.followTrajectory(midTraj3);
    }
}
