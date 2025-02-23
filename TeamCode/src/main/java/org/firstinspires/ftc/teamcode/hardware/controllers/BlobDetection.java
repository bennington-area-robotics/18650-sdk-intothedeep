package org.firstinspires.ftc.teamcode.hardware.controllers;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Blob Detection Demo")
public class BlobDetection extends LinearOpMode {
    private OpenCvCamera camera;
    private BlobDetectionPipeline pipeline;
    private Servo clawServo;

    // Camera resolution
    public static final int CAMERA_WIDTH = 640;
    public static final int CAMERA_HEIGHT = 480;



    @Override
    public void runOpMode() {
        // Initialize camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new BlobDetectionPipeline();
        camera.setPipeline(pipeline);

        // Initialize claw servo
        clawServo = hardwareMap.servo.get("claw_servo");

        // Start camera streaming
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            // Get latest blob position
            Point blobCenter = pipeline.getBlobCenter();

            if (blobCenter != null) {
                // Convert blob position to servo position
                double servoPosition = mapBlobToServo(blobCenter.x);
                clawServo.setPosition(servoPosition);

                telemetry.addData("Blob X", blobCenter.x);
                telemetry.addData("Blob Y", blobCenter.y);
                telemetry.addData("Servo Position", servoPosition);
            } else {
                telemetry.addData("Status", "No blob detected");
            }

            telemetry.update();
        }
    }

    private double mapBlobToServo(double blobX) {
        // Map blob X position (0 to CAMERA_WIDTH) to servo position (0 to 1)
        return Math.min(Math.max(blobX / CAMERA_WIDTH, 0), 1);
    }
}

class BlobDetectionPipeline extends OpenCvPipeline {
    private Mat processedMat = new Mat();
    private Point blobCenter = null;
    // HSV color thresholds for blob detection (adjust these for your samples)
    public static final Scalar LOWER_BOUND = new Scalar(20, 100, 100);  // Lower HSV bounds
    public static final Scalar UPPER_BOUND = new Scalar(30, 255, 255);  // Upper HSV bounds

    @Override
    public Mat processFrame(Mat input) {
        // Convert to HSV color space
        Imgproc.cvtColor(input, processedMat, Imgproc.COLOR_RGB2HSV);

        // Create mask using color thresholds
        Mat mask = new Mat();
        Core.inRange(processedMat, LOWER_BOUND, UPPER_BOUND, mask);

        // Apply morphological operations to remove noise
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find largest contour (blob)
        if (!contours.isEmpty()) {
            MatOfPoint largestContour = contours.get(0);
            double maxArea = Imgproc.contourArea(largestContour);

            for (int i = 1; i < contours.size(); i++) {
                double area = Imgproc.contourArea(contours.get(i));
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contours.get(i);
                }
            }

            // Calculate centroid of largest contour
            Moments moments = Imgproc.moments(largestContour);
            blobCenter = new Point(
                    moments.m10 / moments.m00,
                    moments.m01 / moments.m00
            );

            // Draw contour and centroid for visualization
            Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(0, 255, 0), 2);
            Imgproc.circle(input, blobCenter, 5, new Scalar(255, 0, 0), -1);
        } else {
            blobCenter = null;
        }

        mask.release();
        kernel.release();
        hierarchy.release();

        return input;
    }

    public Point getBlobCenter() {
        return blobCenter;
    }
}