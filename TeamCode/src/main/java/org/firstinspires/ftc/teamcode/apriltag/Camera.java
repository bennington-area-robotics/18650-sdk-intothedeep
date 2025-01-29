package org.firstinspires.ftc.teamcode.apriltag;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.teamcode.hardware.drive.Pose;

public class Camera {
    private final CameraName passable;
    private final String name;
    private final Pose pose;

    public Camera(HardwareMap hardwareMap, String name, Pose pose){
        this.passable = hardwareMap.get(CameraName.class, name);
        this.name = name;
        this.pose = pose;
    }

    public CameraName passable(){
        return passable;
    }

    public String getName(){
        return name;
    }

    public Pose getPose(){
        return pose;
    }
}
