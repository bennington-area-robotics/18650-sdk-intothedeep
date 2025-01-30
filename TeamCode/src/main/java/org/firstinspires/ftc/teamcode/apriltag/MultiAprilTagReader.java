package org.firstinspires.ftc.teamcode.apriltag;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.drive.Pose;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class MultiAprilTagReader {
    private final List<AprilTagProcessor> processors;
    private final List<VisionPortal> portals;

    public MultiAprilTagReader(List<Camera> cameras){
        processors = new ArrayList<>(cameras.size());
        portals = new ArrayList<>(cameras.size());
        int[] viewIds = VisionPortal.makeMultiPortalView(cameras.size(), VisionPortal.MultiPortalLayout.VERTICAL);

        for (int i = 0; i < cameras.size(); i++) {
            Camera camera = cameras.get(i);
            Pose cameraPose = camera.getPose();
            CameraName camName = camera.passable();

            processors.set(i, new AprilTagProcessor.Builder()
                    .setCameraPose(cameraPose.getPosition(), cameraPose.getAngles())
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    .build()
            );

            portals.set(i, new VisionPortal.Builder()
                    .setCamera(camName)
                    .setLiveViewContainerId(viewIds[i])
                    .addProcessor(processors.get(i))
                    .setCameraResolution(camera.getResolution())
                    .build()
            );
        }
    }

    public List<Detection> getAllUniqueDetections(){
        List<Detection> out = new ArrayList<>();
        for (int i = 0; i < processors.size(); i++) {
            out.addAll(getDetections(i));
        }

        return out.stream().distinct().collect(Collectors.toList());
    }

    public List<Detection> getDetections(int cameraNum){
        return processors.get(cameraNum).getDetections().stream().map(Detection::new).collect(Collectors.toList());
    }

    public List<VisionPortal> getPortals() {
        return portals;
    }

    public List<AprilTagProcessor> getProcessors() {
        return processors;
    }
}
