package org.firstinspires.ftc.teamcode.apriltag;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.apriltag.AprilTagReader;
import org.firstinspires.ftc.teamcode.apriltag.AprilTagReaderTest;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;

@TeleOp(name = "AprilTag Position Test", group = "Test")
public class AprilTagPositionTestOpMode extends LinearOpMode {

    private AprilTagReaderTest aprilTagSubsystem;

    @Override
    public void runOpMode() {
        // Initialize the subsystem with the webcam name from your robot configuration
        aprilTagSubsystem = new AprilTagReaderTest(hardwareMap, "Webcam Left");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Get all current detections
            List<AprilTagDetection> detections = aprilTagSubsystem.getDetections();
            telemetry.addData("# AprilTags Detected", detections.size());

            // If we have detections, show the pose information for each
            if (!detections.isEmpty()) {
                for (AprilTagDetection detection : detections) {
                    telemetry.addLine(String.format("\nID %d", detection.id));

                    // Get position data
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));

                    // Get orientation data
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                }
            } else {
                telemetry.addLine("No AprilTags detected");
            }

            // Check for a specific tag (e.g., tag ID 1)
            if (aprilTagSubsystem.isTagVisible(1)) {
                Pose3D tagPose = aprilTagSubsystem.getTagPose(1);
                telemetry.addLine("\nTag ID 1 found!");
                telemetry.addData("Distance", "%.2f inches",
                        Math.sqrt(
                                Math.pow(tagPose.getPosition().x, 2) +
                                        Math.pow(tagPose.getPosition().y, 2) +
                                        Math.pow(tagPose.getPosition().z, 2)
                        ));
            }

            // Get the closest tag (useful for autonomous targeting)
            AprilTagDetection closest = aprilTagSubsystem.getClosestTag();
            if (closest != null) {
                telemetry.addLine("\nClosest tag:");
                telemetry.addData("ID", closest.id);
                telemetry.addData("X", "%.1f", closest.robotPose.getPosition().x);
                telemetry.addData("Y", "%.1f", closest.robotPose.getPosition().y);
                telemetry.addData("Z", "%.1f", closest.robotPose.getPosition().z);
                Pose2D closest2D = aprilTagSubsystem.getClosestTagPose2d();

                telemetry.addLine(String.format("X Y Yaw %6.1f %6.1f %6.1f  (deg)",
                        closest2D.getX(DistanceUnit.INCH),
                        closest2D.getY(DistanceUnit.INCH),
                        closest2D.getHeading(AngleUnit.DEGREES)));
            }

            // Allow toggling camera streaming to save resources
            if (gamepad1.dpad_down) {
                aprilTagSubsystem.stopStreaming();
                telemetry.addLine("Camera streaming stopped");
            } else if (gamepad1.dpad_up) {
                aprilTagSubsystem.resumeStreaming();
                telemetry.addLine("Camera streaming resumed");
            }

            // Change decimation settings with gamepad
            if (gamepad1.x) {
                // Higher accuracy, longer range, lower FPS
                aprilTagSubsystem.setDecimation(1);
                telemetry.addLine("Decimation set to 1 (best accuracy)");
            } else if (gamepad1.y) {
                // Balanced setting
                aprilTagSubsystem.setDecimation(2);
                telemetry.addLine("Decimation set to 2 (balanced)");
            } else if (gamepad1.b) {
                // Lower accuracy, shorter range, higher FPS
                aprilTagSubsystem.setDecimation(3);
                telemetry.addLine("Decimation set to 3 (best FPS)");
            }

            telemetry.addLine("\nControls:");
            telemetry.addLine("DPAD Up/Down: Resume/Stop camera");
            telemetry.addLine("X/Y/B: Change decimation (accuracy vs speed)");



            telemetry.update();
            sleep(50);
        }

        // Make sure to close the vision portal when done
        aprilTagSubsystem.close();
    }
}