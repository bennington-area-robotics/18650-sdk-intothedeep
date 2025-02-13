package org.firstinspires.ftc.teamcode.apriltag;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.teamcode.components.drive.Pose;
import org.firstinspires.ftc.teamcode.hardware.SmartCamera;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;
import java.util.Optional;
import java.util.stream.Collectors;

@Config
public class AprilTagReader {


    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    //todo test different values of decimation also test different PoseSolvers

    private boolean isInitialized = false;

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;



    public AprilTagReader(SmartCamera camera){

        if(!isInitialized){
            aprilTag = new AprilTagProcessor.Builder()
                    .setCameraPose(camera.getPosition(), camera.getAngles())
                    .build();

            assert aprilTag != null;

            VisionPortal.Builder builder = new VisionPortal.Builder();

            if (USE_WEBCAM) {
                builder.setCamera(camera.passable());
            } else {
                builder.setCamera(BuiltinCameraDirection.BACK);
            }

            builder.setCameraResolution(camera.getResolution());

            builder.addProcessor(aprilTag);

            // Build the Vision Portal, using the above settings.
            visionPortal = builder.build();

            isInitialized = true;
        }
    }

    public String getDetectionString(){
        Optional<Pose> poseOptional = getFirstPose();
        if(poseOptional.isPresent()){
            Pose pose = poseOptional.get();
            return String.format(Locale.ENGLISH, "X Y Heading %6.1f %6.1f %6.1f  (inch)",
                    pose.x(),
                    pose.y(),
                    pose.heading());
        }else{
            return "No detections";
        }
    }


    /**
     * @return an optional pose based on the first detection found.
     */
    public Optional<Pose> getFirstPose(){
        try{
            Detection detection = getDetections().get(0);
            Pose localization = detection.getRobotPose();

            return Optional.of(localization);
        } catch(IndexOutOfBoundsException e) {
            return Optional.empty();
        }
    }

    /**
     * @return the detections found by the processor.
     */
    public List<Detection> getDetections (){
        return aprilTag.getDetections().stream().map(Detection::new).collect(Collectors.toList());
    }

    /**
     * @return whether the processor detects any april tags.
     */
    public boolean hasDetections(){
        return !aprilTag.getDetections().isEmpty();
    }

}
