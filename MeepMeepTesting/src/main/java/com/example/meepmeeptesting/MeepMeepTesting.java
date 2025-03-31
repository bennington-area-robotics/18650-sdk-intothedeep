package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static double startX = -0;
    public static double startY = 63;
    public static double startAng = -90;
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(startX, startY, Math.toRadians(startAng));



        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40.05530633326986, 40.05530633326986, Math.toRadians(163.2), Math.toRadians(166.9090909090909), 12.96)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(0, 63, Math.toRadians(90)))
                        .back(25)
                        .forward(3)
                        //.splineTo(new Vector2d(0, 50), Math.toRadians(45))
                        .splineToSplineHeading(new Pose2d(-24, 55, Math.toRadians(0)), Math.toRadians(180))
                        .strafeLeft(10)
                        .splineToConstantHeading(new Vector2d(-24, 55), Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(0, 38, Math.toRadians(90)), Math.toRadians(-90))
                        //.splineToLinearHeading(new Pose2d(-12, 40, Math.toRadians(-90)), Math.toRadians(-90))
                        //.splineTo(new Vector2d(55, 55), Math.toRadians(45))
                        //push to red observation zone
                        /*.splineTo(new Vector2d(0, -45), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(40, -48), Math.toRadians(90))
                        .splineTo(new Vector2d(40,-20), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(48,-12), Math.toRadians(-90))

                        .splineToConstantHeading(new Vector2d(48, -55), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(48, -20), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(59, -12), Math.toRadians(-90))

                        .splineToConstantHeading(new Vector2d(59, -55), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(59, -20), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(70, -12), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(70, -55), Math.toRadians(-90))*/


                        //push to red net zone
                        /*
                        .splineTo(new Vector2d(0, -45), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-38, -48), Math.toRadians(90))
                        .splineTo(new Vector2d(-38,-20), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-48,-12), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(-48, -40), Math.toRadians(-90))
                        .splineTo(new Vector2d(-54, -60), Math.toRadians(-45))
                        .splineToConstantHeading(new Vector2d(-48, -40), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(-48, -12, Math.toRadians(90)), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-59, -12), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(-59, -60), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(-59, -20), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-68, -12), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(-68, -60), Math.toRadians(-90))
                        */


                        //pushes to blue net zone
                        /*.splineTo(new Vector2d(0, 45), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(38, 48), Math.toRadians(-90))
                        .splineTo(new Vector2d(38,20), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(48,12), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(48, 40), Math.toRadians(90))
                        .splineTo(new Vector2d(54, 60), Math.toRadians(45))
                        .splineToConstantHeading(new Vector2d(48, 40), Math.toRadians(-90))
                        .splineToSplineHeading(new Pose2d(48, 12, Math.toRadians(-90)), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(59, 12), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(59, 60), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(59, 20), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(68, 12), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(68, 60), Math.toRadians(90))


                         */
                        //pushes to blue observation zone
                        /*.splineTo(new Vector2d(0, 45), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(-40, 48), Math.toRadians(-90))
                        .splineTo(new Vector2d(-40,20), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(-48,12), Math.toRadians(90))

                        .splineToConstantHeading(new Vector2d(-48, 55), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-48, 20), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(-59, 12), Math.toRadians(90))

                        .splineToConstantHeading(new Vector2d(-59, 55), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-59, 20), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(-70, 12), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-70, 55), Math.toRadians(-90))*/
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}