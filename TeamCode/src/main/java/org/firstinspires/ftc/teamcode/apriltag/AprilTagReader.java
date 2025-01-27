package org.firstinspires.ftc.teamcode.apriltag;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.OpModeCore;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;
import java.util.Optional;
import java.util.stream.Collectors;

@Config
public class AprilTagReader {

    //todo add support for swapping between multiple cameras
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    public static int decimation = 0; //todo test different values of decimation also test different PoseSolvers
    public static Size resolution = new Size(640, 480);
    private final Position cameraPosition;
    private final YawPitchRollAngles cameraOrientation;
    private boolean isInitialized = false;

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;



    public AprilTagReader(HardwareMap hardwareMap, Position cameraPosition, YawPitchRollAngles cameraOrientation, Size resolution, int decimation){

        this.cameraPosition = cameraPosition;
        this.cameraOrientation = cameraOrientation;
        AprilTagReader.resolution = resolution;
        AprilTagReader.decimation = decimation;

        if(!isInitialized){
            aprilTag = new AprilTagProcessor.Builder()
                    .setCameraPose(cameraPosition, cameraOrientation)
                    .build();

            assert aprilTag != null;

            VisionPortal.Builder builder = new VisionPortal.Builder();

            if (USE_WEBCAM) {
                builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            } else {
                builder.setCamera(BuiltinCameraDirection.BACK);
            }

            builder.setCameraResolution(resolution);

            builder.addProcessor(aprilTag);

            // Build the Vision Portal, using the above settings.
            visionPortal = builder.build();

            isInitialized = true;
        }

        OpModeCore.getTelemetry().addData("April Tag", () -> {
            Optional<Pose2D> poseOptional = getFirstPose();
            if(poseOptional.isPresent()){
                Pose2D pose = poseOptional.get();
                return String.format(Locale.ENGLISH, "X Y Heading %6.1f %6.1f %6.1f  (inch)",
                        pose.getX(DistanceUnit.INCH),
                        pose.getY(DistanceUnit.INCH),
                        pose.getHeading(AngleUnit.DEGREES));
            }else{
                return "No detections";
            }
        });
    }


    /**
     * @return an optional pose based on the first detection found.
     */
    public Optional<Pose2D> getFirstPose(){
        try{
            Detection detection = getDetections().get(0);
            Pose2D localization = detection.getRobotPose2D();

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
