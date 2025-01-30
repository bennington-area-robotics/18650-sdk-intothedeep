package org.firstinspires.ftc.teamcode.apriltag;

import android.graphics.ImageFormat;

import com.qualcomm.robotcore.hardware.HardwareMap;

import android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.hardware.drive.Pose;

public class Camera {
    private final CameraName passable;
    private final String name;
    private final Pose pose;

    /**
     * @param hardwareMap HardwareMap to get the CameraName from.
     * @param name configured name of the camera to create.
     * @param pose pose of the camera relative to the robot. The pitch is adjusted -90 degrees for the processor.
     */
    public Camera(HardwareMap hardwareMap, String name, Pose pose){
        this.passable = hardwareMap.get(CameraName.class, name);
        this.name = name;
        this.pose = pose.plusPitch(-90);
    }

    public CameraName passable(){
        return passable;
    }

    public Position getPosition(){
        return pose.getPosition();
    }

    public YawPitchRollAngles getAngles(){
        return pose.getAngles();
    }

    public String getName(){
        return name;
    }

    public Pose getPose(){
        return pose;
    }

    public Size getResolution(){
        org.firstinspires.ftc.robotcore.external.android.util.Size ftcSize = passable.getCameraCharacteristics().getDefaultSize(ImageFormat.YUV_420_888);
        return new Size(ftcSize.getWidth(), ftcSize.getHeight());
    }
}
