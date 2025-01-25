package org.firstinspires.ftc.teamcode.apriltag;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * Provides a simpler and more concise interaction API for AprilTagDetections.
 */
public class Detection {
    AprilTagDetection tagDetection;

    public Detection(AprilTagDetection aprilTagDetection) {
        this.tagDetection = aprilTagDetection;
    }

    /**
     * @return the absolute 3D pose of the robot based on the april tag detection.
     */
    public Pose3D getRobotPose3D(){
        return tagDetection.robotPose;
    }


    //todo confirm the ori yaw is actual absolute heading.
    /**
     * @return the absolute 2D pose of the robot based on the april tag detection.
     */
    public Pose2D getRobotPose2D(){
        Position pos = tagDetection.robotPose.getPosition();
        YawPitchRollAngles ori = tagDetection.robotPose.getOrientation();
        return new Pose2D(pos.unit, pos.x, pos.y, AngleUnit.DEGREES, ori.getYaw(AngleUnit.DEGREES));
    }

    public String getName(){
        return tagDetection.metadata.name;
    }

    public int getId(){
        return tagDetection.metadata.id;
    }
}
