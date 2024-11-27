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

    public static double startX = -48;
    public static double startY = 48;
    public static double startAng = -90;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(startX, startY, Math.toRadians(startAng));
        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                //.lineToConstantHeading(new Vector2d(-48, -3.744))
                .forward(48*0.92)
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                //.lineToConstantHeading(new Vector2d(-48, -3.744))
                .back(48*0.92)
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                //.lineToConstantHeading(new Vector2d(-48, -3.744))
                .strafeLeft(10)
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                //.lineToConstantHeading(new Vector2d(-48, -3.744))
                .forward(48*0.92)
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                //.lineToConstantHeading(new Vector2d(-48, -3.744))
                .back(48*0.92)
                .build();
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                //.lineToConstantHeading(new Vector2d(-48, -3.744))
                .strafeLeft(10)
                .build();
        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                //.lineToConstantHeading(new Vector2d(-48, -3.744))
                .forward(48*0.92)
                .build();
        /*
        Trajectory midTraj = drive.trajectoryBuilder(trajToBars.end(), true)
                .splineToConstantHeading(new Vector2d(-35.5, 48), 0  )
                .build();

        Trajectory midTraj2 = drive.trajectoryBuilder(midTraj.end())
                .lineToLinearHeading(new Pose2d(-48, 0, Math.toRadians(180)))
                .build();
        Trajectory midTraj3 = drive.trajectoryBuilder(midTraj2.end())
                .lineToSplineHeading(new Pose2d(30,0))
                .build();
        */

        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
        drive.followTrajectory(traj5);
        drive.followTrajectory(traj6);
        drive.followTrajectory(traj7);

    }
}
