package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.hardware.drive.ConfiguredMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.drive.DriveBase;
import org.firstinspires.ftc.teamcode.hardware.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.hardware.drive.Pose;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Config
@Autonomous(group = "drive")
public class AutonomousCore extends LinearOpMode {

    //TODO FOR EBEN - clean up this code! remove the unnecessary code if its commented, implement the methods I added here

    public static double startX = 0;
    public static double startY = 63;
    public static double startAng = 90;

    public static boolean DELIVER_SPECIMEN = true;
    public static AutoTask task = AutoTask.PUSH_SAMPLES_TO_OBSERVATION_ZONE;

    public enum AutoTask {
        PUSH_SAMPLES_TO_OBSERVATION_ZONE, PUSH_SAMPLES_TO_NET_ZONE, PARK_IN_OBSERVATION_ZONE, PARK_LEVEL_1_ASCENT
    }

    private DriveBase drive;
    private final Pose2d startPose = new Pose(startX, startY, startAng).toRR();

    private final Pose2d lastEndPose = startPose;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        run();
    }

    /**
     * Prepare the robot for autonomous
     * @implNote This should account for where the robot starts, and the position all 'appendages' start in.
     * If there is some initialization movement that needs to happen to allow the bot to start in a position where it will fit in an 18x18, this is when it should happen.
     * Since we get to define the starting position, this is a good time to reset our encoders if necessary.
     */
    public void initialize(){
        drive = new DriveBase(hardwareMap);
        drive.setPoseEstimate(startPose);

        //TODO FOR EBEN - finish implementing this
    }

    /**
     * Complete delivering a sample (if true) and the AutoTask.
     * @implNote This should follow a unique path depending on DELIVER_SPECIMEN and the AutoTask.
     */
    public void run(){
        //TODO FOR EBEN - finish implementing this

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

    //todo add methods, each corresponding to a move you want to make in autonomous
}
