package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static double startX = -11.5;
    public static double startY = 63;
    public static double startAng = -90;
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(startX, startY, Math.toRadians(startAng));



        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40.05530633326986, 40.05530633326986, Math.toRadians(163.2), Math.toRadians(166.9090909090909), 12.96)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-11.5, 63, Math.toRadians(-90)))
                        .splineTo(new Vector2d(-36, 48), Math.toRadians(180))
                        .lineToLinearHeading(new Pose2d(-36, 12,Math.toRadians(90)))
                        .strafeLeft(12)
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}