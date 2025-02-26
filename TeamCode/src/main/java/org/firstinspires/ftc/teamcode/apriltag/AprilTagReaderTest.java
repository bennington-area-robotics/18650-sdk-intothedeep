package org.firstinspires.ftc.teamcode.apriltag;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.HashMap;
import java.util.Map;

/**
 * AprilTagSubsystem - A subsystem for detecting and tracking AprilTags
 *
 * This subsystem handles:
 * - AprilTag detection initialization
 * - Camera positioning and configuration
 * - Retrieval of Pose3d data from AprilTag detections
 */
@Config
public class AprilTagReaderTest {

    public static double yaw = 90, pitch = -90, roll =0;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private String webcamName;

    // Position and orientation of the camera on the robot
    private Position cameraPosition;
    private YawPitchRollAngles cameraOrientation;

    // Store the last known pose for each detected tag
    private Map<Integer, Pose3D> lastDetectedPoses = new HashMap<>();

    private Pose2D robotPose;

    /**
     * Constructor with default camera positioning
     *
     * @param hardwareMap  The OpMode's hardwareMap
     * @param webcamName   The configured name of the webcam in the hardwareMap
     */
    public AprilTagReaderTest(HardwareMap hardwareMap, String webcamName) {
        this(hardwareMap, webcamName,
                new Position(DistanceUnit.INCH, 0, 0, 0, 0),
                new YawPitchRollAngles(AngleUnit.DEGREES, yaw, pitch, roll, 0));
    }

    /**
     * Constructor with custom camera positioning
     *
     * @param hardwareMap       The OpMode's hardwareMap
     * @param webcamName        The configured name of the webcam in the hardwareMap
     * @param cameraPosition    Position of the camera relative to robot center
     * @param cameraOrientation Orientation of the camera on the robot
     */
    public AprilTagReaderTest(HardwareMap hardwareMap, String webcamName,
                             Position cameraPosition, YawPitchRollAngles cameraOrientation) {
        this.webcamName = webcamName;
        this.cameraPosition = cameraPosition;
        this.cameraOrientation = cameraOrientation;

        initAprilTagDetection(hardwareMap);
    }

    /**
     * Initialize the AprilTag processor and vision portal
     */
    private void initAprilTagDetection(HardwareMap hardwareMap) {
        // Create the AprilTag processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                // Configure with the camera's position and orientation on the robot
                .setCameraPose(cameraPosition, cameraOrientation)
                // Uncomment and adjust these settings as needed for your robot
                //.setDrawAxes(true)
                //.setDrawCubeProjection(true)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                .build();

        // Set decimation for better performance vs range tradeoff
        // Lower = better range but slower FPS, Higher = shorter range but faster FPS
        aprilTagProcessor.setDecimation(2);

        // Create the vision portal
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, webcamName));
        builder.addProcessor(aprilTagProcessor);
        visionPortal = builder.build();
    }

    /**
     * Get all current AprilTag detections
     *
     * @return List of all current detections
     */
    public List<AprilTagDetection> getDetections() {
        return aprilTagProcessor.getDetections();
    }

    /**
     * Get the 2D pose of the closest AprilTag detection
     *
     * @return The 2D pose of the closest detection, or null if none found
     */
    public Pose2D getClosestTagPose2d() {
        AprilTagDetection closest = getClosestTag();

        if (closest == null) {
            return null;
        }

        // Extract x, y, and heading from the 3D pose
        double x = closest.robotPose.getPosition().x;
        double y = closest.robotPose.getPosition().y;
        double heading = closest.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

        return new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading);
    }

    /**
     * Get the pose of a specific AprilTag by ID
     *
     * @param tagId The ID of the tag to find
     * @return The pose if found, null otherwise
     */
    public Pose3D getTagPose(int tagId) {
        // Look for the tag in current detections
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == tagId) {
                // Store this pose in our map and return it
                lastDetectedPoses.put(tagId, detection.robotPose);
                return detection.robotPose;
            }
        }

        // If not found in current detections, return the last known pose if available
        return lastDetectedPoses.get(tagId);
    }

    /**
     * Check if a specific AprilTag is currently visible
     *
     * @param tagId The ID of the tag to check
     * @return true if the tag is currently detected
     */
    public boolean isTagVisible(int tagId) {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == tagId) {
                return true;
            }
        }

        return false;
    }

    /**
     * Get the closest AprilTag detection (by distance)
     *
     * @return The closest detection, or null if none found
     */
    public AprilTagDetection getClosestTag() {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

        if (currentDetections.isEmpty()) {
            return null;
        }

        AprilTagDetection closest = null;
        double closestDistance = Double.MAX_VALUE;

        for (AprilTagDetection detection : currentDetections) {
            // Calculate distance to this tag
            double distance = Math.sqrt(
                    Math.pow(detection.robotPose.getPosition().x, 2) +
                            Math.pow(detection.robotPose.getPosition().y, 2)
            );

            if (distance < closestDistance) {
                closestDistance = distance;
                closest = detection;
            }
        }

        return closest;
    }

    /**
     * Start or resume streaming from the camera
     */
    public void resumeStreaming() {
        if (visionPortal != null) {
            visionPortal.resumeStreaming();
        }
    }

    @Override
    public String toString() {
        Pose2D closestPose = getClosestTagPose2d();

        if (closestPose == null) {
            return "No AprilTag detected";
        }

        return String.format("X Y Yaw %6.1f %6.1f %6.1f (deg)",
                closestPose.getX(DistanceUnit.INCH),
                closestPose.getY(DistanceUnit.INCH),
                closestPose.getHeading(AngleUnit.DEGREES));
    }

    /**
     * Stop streaming from the camera (to save CPU resources)
     */
    public void stopStreaming() {
        if (visionPortal != null) {
            visionPortal.stopStreaming();
        }
    }

    /**
     * Close the vision portal when no longer needed
     */
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /**
     * Check if the vision system is currently active
     *
     * @return true if the vision portal is active
     */
    public boolean isActive() {
        return visionPortal != null && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    /**
     * Set the decimation factor for AprilTag detection
     *
     * @param decimation Value from 1-4 (1=best range/slowest, 3-4=shortest range/fastest)
     */
    public void setDecimation(int decimation) {
        if (aprilTagProcessor != null) {
            aprilTagProcessor.setDecimation(decimation);
        }
    }
}
