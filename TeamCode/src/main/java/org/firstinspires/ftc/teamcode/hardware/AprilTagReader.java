package org.firstinspires.ftc.teamcode.hardware;

import android.annotation.SuppressLint;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.OpModeCore;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Optional;
@Config
@TeleOp(name = "Concept: AprilTag Localization", group = "Concept")
public class AprilTagReader {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    public static int decimation = 0;
    public static Size resolution = new Size(640, 480);
    private Position cameraPosition;
    private YawPitchRollAngles cameraOrientation;

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
        this.resolution = resolution;
        decimation = decimation;

        this.initAprilTag(hardwareMap);

    }
    /**
     * Initialize the AprilTag processor.
     */
    @SuppressLint("DefaultLocale")
    private void initAprilTag(HardwareMap hardwareMap) {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        aprilTag.setDecimation(decimation);

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTag);

        visionPortal = builder.build();

        OpModeCore.getTelemetry().addData("April Tag", () -> {
            Optional<Pose2D> poseOptional = getFirstPosition();
            if(poseOptional.isPresent()){
                Pose2D pose = poseOptional.get();
                return String.format("X Y Heading %6.1f %6.1f %6.1f  (inch)",
                        pose.getX(DistanceUnit.INCH),
                        pose.getY(DistanceUnit.INCH),
                        pose.getHeading(AngleUnit.RADIANS));
            }else{
                return "None detected";
            }
        });
    }

    public Optional<Pose2D> getFirstPosition(){
        try{
            Pose3D position = getAprilTags().get(0).robotPose;
            Pose2D localization = new Pose2D(DistanceUnit.INCH, position.getPosition().x, position.getPosition().y, AngleUnit.RADIANS, position.getOrientation().getYaw(AngleUnit.RADIANS));

            return Optional.of(localization);
        } catch(IndexOutOfBoundsException e) {
            return Optional.empty();
        }

    }

    /**
     * Add telemetry about AprilTag detections.
     */
    public List<AprilTagDetection> getAprilTags (){
        return aprilTag.getDetections();
    }

}
