package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.apriltag.AprilTagReader;
import org.firstinspires.ftc.teamcode.apriltag.Camera;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Collector;
import org.firstinspires.ftc.teamcode.hardware.drive.ConfiguredMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.drive.DriveBase;
import org.firstinspires.ftc.teamcode.hardware.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.hardware.drive.Pose;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Config
@Autonomous(group = "drive", name="AutonomousNetDeliveryPath")
public class AutonomousNetDelivery extends LinearOpMode {

    //TODO FOR EBEN - clean up this code! remove the unnecessary code if its commented, implement the methods I added here

    public static double blueStartX = 23.5;
    public static double blueStartY = 63;
    public static double blueStartAng = 90;

    public static double redStartX = 12;
    public static double redStartY = -63;
    public static double redStartAng = -90;
    public static TrajectoryVelocityConstraint velocityConstraint = ConfiguredMecanumDrive.getVelocityConstraint(
            30,
            2,
            DriveConstants.TRACK_WIDTH);

    public static TrajectoryAccelerationConstraint accelerationConstraint = ConfiguredMecanumDrive.getAccelerationConstraint(
            20);

    private static AutonomousCore instance;
    ElapsedTime tickTimer;
    private static Arm arm;
    private static Collector collector;
    private PrettyTelemetry prettyTelem;

    public static AutonomousCore getInstance(){
        return instance;
    }

    public static Telemetry getTelemetry(){
        return instance.telemetry;
    }

    public static boolean DELIVER_SPECIMEN = true;
    public static AutoTask task = AutoTask.PUSH_SAMPLES_TO_OBSERVATION_ZONE;

    public enum AutoTask {
        PUSH_SAMPLES_TO_OBSERVATION_ZONE, PUSH_SAMPLES_TO_NET_ZONE, PARK_IN_OBSERVATION_ZONE, PARK_LEVEL_1_ASCENT
    }
    private AprilTagReader aprilTagReader;
    private DriveBase drive;
    private final Pose2d blueStartPose = new Pose(blueStartX, blueStartY, blueStartAng).toRR();
    private final Pose2d redStartPose = new Pose(redStartX, redStartY, redStartAng).toRR();



    //private final Pose2d lastEndPose = startPose;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();


        waitForStart();

        run();
    }

    /**
     * Prepare the robot for autonomous
     * @implNote This should account for where the robot starts, and the position all 'appendages' start in.
     * If there is some initialization movement that needs to happen to allow the bot to start in a position where it will fit in an 18x18, this is when it should happen.
     * Since we get to define the starting position, this is a good time to reset our encoders if necessary.
     */
    private void configureTelemetry(){
        prettyTelem = new PrettyTelemetry(telemetry);

        prettyTelem.addLine("System Status")
                .addData("Localization: ", () -> drive.getPoseSimple())
        ;
        ;

        prettyTelem.addLine("Arm Status")
                .addData("Current Angle", () -> arm.getAngle())
                .addData("Target Angle", () -> arm.getTargetAngle())
                .addData("Current Extension", () -> arm.getExtension())
                .addData("Target Extension", () -> arm.getTargetExtension())
                .addData("Last Angle Power", () -> arm.getLastAnglePower())
                .addData("Last Extension Power", () -> arm.getLastExtensionPower())
                .addData("Tilt Limit Sensor Pressed?", () -> arm.tiltLimitSensor.isPressed())
                .addData("Extension Limit Sensor Pressed?", () -> arm.extensionLimitSensor.isPressed());

        prettyTelem.addLine("Grip")
                .addData("Position", () -> collector.getGripPosition())
                .addData("Open?", () -> collector.isGripOpen())
                .addData("Closed?", () -> collector.isGripClosed());

        prettyTelem.addLine("Wrist")
                .addData("Position", () -> collector.getWristAngle())
                .addData("Target", collector::getWristTarget)
                .addData("Up?", () -> collector.isWristUp())
                .addData("Down?", () -> collector.isWristDown());


        prettyTelem.addData("April Tag", () -> aprilTagReader.getFirstPose().toString());
    }

    public void initializeStartingPosition(){
        arm.moveToTargetAngleBlocking(36, this::tick);
        collector.moveWristToBlocking(240, this::tick, true);
        collector.closeGrip();
        arm.setAnglePower(0);
    }
    public void initialize(){
        drive = new DriveBase(hardwareMap);
        drive.setPoseEstimate(blueStartPose);

        arm = new Arm(
                hardwareMap,
                "tiltMotorLeft",
                "tiltMotorRight",
                "extensionMotor",
                "tiltLimitSensor",
                "extensionLimitSensor",
                this);
        collector = new Collector(
                arm,
                hardwareMap,
                "colorSensor",
                "wristMotor",
                "gripServo",
                "wristServo");
        tickTimer = new ElapsedTime();
        aprilTagReader = new AprilTagReader(
                new Camera(
                        hardwareMap,
                        "Webcam Right",
                        new Pose(0, 0, 0)
                )
        );

        configureTelemetry();
        initializeStartingPosition();

        //TODO FOR EBEN - finish implementing this
    }

    public void blueNetZoneSamplePushPath(){
        TrajectoryVelocityConstraint velocityConstraint = ConfiguredMecanumDrive.getVelocityConstraint(
                25,
                2,
                DriveConstants.TRACK_WIDTH);

        TrajectoryAccelerationConstraint accelerationConstraint = ConfiguredMecanumDrive.getAccelerationConstraint(
                15);
        Trajectory pushSamples = drive.trajectoryBuilder(blueStartPose, true)
                //.splineTo(new Vector2d(0, 45), Math.toRadians(-90))
                //.splineToConstantHeading(new Vector2d(38, 48), Math.toRadians(-90))
                .splineTo(
                        new Vector2d(38, 45),
                        Math.toRadians(-90),
                        velocityConstraint,
                        accelerationConstraint)
                .splineTo(
                        new Vector2d(38,20),
                        Math.toRadians(-90),
                        velocityConstraint,
                        accelerationConstraint)
                .splineToConstantHeading(new Vector2d(45,12), Math.toRadians(90), velocityConstraint, accelerationConstraint)
                .splineToConstantHeading(new Vector2d(45, 40), Math.toRadians(90),velocityConstraint, accelerationConstraint)
                .splineTo(new Vector2d(52, 57), Math.toRadians(45), velocityConstraint, accelerationConstraint)
                .build();

        Trajectory pushSamples2 = drive.trajectoryBuilder(pushSamples.end(), true)
                .splineToConstantHeading(new Vector2d(45, 40), Math.toRadians(-90), velocityConstraint, accelerationConstraint)
                .splineToSplineHeading(new Pose2d(45, 12, Math.toRadians(90)), Math.toRadians(90), velocityConstraint, accelerationConstraint)
                .splineToConstantHeading(new Vector2d(53, 12), Math.toRadians(90), velocityConstraint, accelerationConstraint)
                .splineToConstantHeading(new Vector2d(53, 60), Math.toRadians(90), velocityConstraint, accelerationConstraint)
                .build();
        Trajectory pushSamples3 = drive.trajectoryBuilder(pushSamples2.end(), true)
                .splineToConstantHeading(new Vector2d(53, 20), Math.toRadians(-90), velocityConstraint, accelerationConstraint)
                .splineToConstantHeading(new Vector2d(60, 12), Math.toRadians(90), velocityConstraint, accelerationConstraint)
                .splineToConstantHeading(new Vector2d(60, 57), Math.toRadians(90), velocityConstraint, accelerationConstraint)
                .build();
        drive.followTrajectories(pushSamples, pushSamples2, pushSamples3);
    }

    public void blueObservationSamplePushPath(){
        TrajectoryVelocityConstraint velocityConstraint = ConfiguredMecanumDrive.getVelocityConstraint(
                25,
                2,
                DriveConstants.TRACK_WIDTH);

        TrajectoryAccelerationConstraint accelerationConstraint = ConfiguredMecanumDrive.getAccelerationConstraint(
                15);

        Trajectory moveRobot = drive.trajectoryBuilder(blueStartPose, true)
                /*.splineTo(
                        new Vector2d(0, 45),
                        Math.toRadians(-90),
                        velocityConstraint,
                        accelerationConstraint)
                .splineToConstantHeading(
                        new Vector2d(-40, 45),
                        Math.toRadians(-90),
                        velocityConstraint,
                        accelerationConstraint)*/
                .splineTo(
                        new Vector2d(-39, 45),
                        Math.toRadians(-90),
                        velocityConstraint,
                        accelerationConstraint)
                .splineTo(
                        new Vector2d(-39,20),
                        Math.toRadians(-90),
                        velocityConstraint,
                        accelerationConstraint)
                .splineToConstantHeading(
                        new Vector2d(-45,12),
                        Math.toRadians(90),
                        velocityConstraint,
                        accelerationConstraint)
                .splineTo(
                        new Vector2d(-45, 55),
                        Math.toRadians(90),
                        velocityConstraint,
                        accelerationConstraint)
                .build();

        Trajectory moveRobot2 = drive.trajectoryBuilder(moveRobot.end(), true)

                .splineTo(new Vector2d(-45, 20), Math.toRadians(-90), velocityConstraint, accelerationConstraint)
                .splineToConstantHeading(new Vector2d(-53, 12), Math.toRadians(90), velocityConstraint, accelerationConstraint)

                .splineTo(new Vector2d(-53, 55), Math.toRadians(90), velocityConstraint, accelerationConstraint)
                .build();

        Trajectory moveRobot3 = drive.trajectoryBuilder(moveRobot2.end(), true)
                .splineTo(new Vector2d(-53, 20), Math.toRadians(-90),velocityConstraint, accelerationConstraint)
                .splineToConstantHeading(new Vector2d(-61, 12), Math.toRadians(90), velocityConstraint, accelerationConstraint)
                .splineTo(new Vector2d(-62, 60), Math.toRadians(90), velocityConstraint, accelerationConstraint)
                .build();
        drive.followTrajectory(moveRobot);
        drive.followTrajectory(moveRobot2);
        drive.followTrajectory(moveRobot3);
    }

    public void redNetZoneSamplePushPath(){
        Trajectory pushSamples = drive.trajectoryBuilder(redStartPose, true)
                .splineTo(new Vector2d(0, -45), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-38, -48), Math.toRadians(90))
                .splineTo(new Vector2d(-38,-20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-48,-12), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-48, -40), Math.toRadians(-90))
                .splineTo(new Vector2d(-54, -60), Math.toRadians(-45))
                .splineToConstantHeading(new Vector2d(-48, -40), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-48, -12, Math.toRadians(90)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-59, -12), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-59, -60), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-59, -20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-68, -12), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-68, -60), Math.toRadians(-90))
                .build();
        drive.followTrajectory(pushSamples);
    }

    public void redObservationZoneSamplePushPath(){
        Trajectory pushSamples = drive.trajectoryBuilder(redStartPose, true)
                .splineTo(new Vector2d(0, -45), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(40, -48), Math.toRadians(90))
                .splineTo(new Vector2d(40,-20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(48,-12), Math.toRadians(-90))

                .splineToConstantHeading(new Vector2d(48, -55), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(48, -20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(59, -12), Math.toRadians(-90))

                .splineToConstantHeading(new Vector2d(59, -55), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(59, -20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(70, -12), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(70, -55), Math.toRadians(-90))
                .build();
        drive.followTrajectory(pushSamples);
    }

    public void linearPushPath(){
        Trajectory path = drive.trajectoryBuilder(blueStartPose, true)
                .lineToLinearHeading(new Pose2d(0, 45, Math.toRadians(90)))
                .build();
        Trajectory path2 = drive.trajectoryBuilder(path.end(), true)
                .lineToLinearHeading(new Pose2d(-40, 45,Math.toRadians(90)))
                .build();
        Trajectory path3 = drive.trajectoryBuilder(path2.end(), true)
                .lineToLinearHeading(new Pose2d(-40, 12, Math.toRadians(90)))
                .build();
        Trajectory path4 = drive.trajectoryBuilder(path3.end(), true)
                .lineToLinearHeading(new Pose2d(-48, 12, Math.toRadians(90)))
                .build();
        Trajectory path5 = drive.trajectoryBuilder(path4.end(), true)
                .lineToLinearHeading(new Pose2d(-48, 60, Math.toRadians(90)))
                .build();
        Trajectory path6 = drive.trajectoryBuilder(path5.end(), true)
                .lineToLinearHeading(new Pose2d(-48, 12, Math.toRadians(90)))
                .build();
        Trajectory path7 = drive.trajectoryBuilder(path6.end(), true)
                .lineToLinearHeading(new Pose2d(-59, 12, Math.toRadians(90)))
                .build();
        Trajectory path8 = drive.trajectoryBuilder(path7.end(), true)
                .lineToLinearHeading(new Pose2d(-59, 60, Math.toRadians(90)))
                .build();
        Trajectory path9 = drive.trajectoryBuilder(path8.end(), true)
                .lineToLinearHeading(new Pose2d(-59, 12, Math.toRadians(90)))
                .build();
        Trajectory path10 = drive.trajectoryBuilder(path9.end(), true)
                .lineToLinearHeading(new Pose2d(-65, 12, Math.toRadians(90)))
                .build();
        Trajectory path11 = drive.trajectoryBuilder(path10.end(), true)
                .lineToLinearHeading(new Pose2d(-65, 60, Math.toRadians(90)))
                .build();

        drive.followTrajectories(path, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11);

    }

    public void sampleDeliveryPath(){
        collector.moveWristToBlocking(0, this::tick, false);
        prettyTelem.addLine("what the fuck");
        prettyTelem.update();
        Trajectory path = drive.trajectoryBuilder(blueStartPose, true)
                .splineToSplineHeading(new Pose2d(40, 57, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(57, 57, Math.toRadians(45)), Math.toRadians(45))
                .build();

        Trajectory pushSamples = drive.trajectoryBuilder(path.end(), true)
                .splineToSplineHeading(
                        new Pose2d(38, 45, Math.toRadians(90)),
                        Math.toRadians(-90),
                        velocityConstraint,
                        accelerationConstraint)
                .splineToSplineHeading(
                        new Pose2d(38,20, Math.toRadians(90)),
                        Math.toRadians(-90),
                        velocityConstraint,
                        accelerationConstraint)
                .splineToConstantHeading(new Vector2d(45,12), Math.toRadians(90), velocityConstraint, accelerationConstraint)
                .splineToConstantHeading(new Vector2d(45, 40), Math.toRadians(90),velocityConstraint, accelerationConstraint)
                .splineTo(new Vector2d(52, 57), Math.toRadians(45), velocityConstraint, accelerationConstraint)
                .build();

        Trajectory pushSamples2 = drive.trajectoryBuilder(pushSamples.end(), true)
                .splineToConstantHeading(new Vector2d(45, 40), Math.toRadians(-90), velocityConstraint, accelerationConstraint)
                .splineToSplineHeading(new Pose2d(45, 12, Math.toRadians(90)), Math.toRadians(90), velocityConstraint, accelerationConstraint)
                .splineToConstantHeading(new Vector2d(53, 12), Math.toRadians(90), velocityConstraint, accelerationConstraint)
                .splineToConstantHeading(new Vector2d(53, 60), Math.toRadians(90), velocityConstraint, accelerationConstraint)
                .build();
        Trajectory moveToAscent = drive.trajectoryBuilder(pushSamples2.end(), true)
                .splineToSplineHeading(new Pose2d(48, 24, Math.toRadians(225)), Math.toRadians(225), velocityConstraint, accelerationConstraint)
                .splineToSplineHeading(new Pose2d(21, 10, Math.toRadians(180)), Math.toRadians(180), velocityConstraint, accelerationConstraint)
                .build();

        drive.followTrajectory(path);
        placeSample();
        arm.moveToTargetExtensionBlocking(20, this::tick);
        drive.followTrajectoryAsync(pushSamples);
        arm.moveToTargetAngleBlocking(45, this::tick);
        arm.setTargetExtension(0);
        while(drive.isBusy()){
            drive.update();
            tick();
        }

        //drive.followTrajectories(pushSamples2);
        drive.followTrajectoryAsync(moveToAscent);
        arm.setTargetAngle(100);
        collector.wristTo(0);
        while(drive.isBusy()){
            drive.update();
            tick();
        }
    }

    public void placeSample(){
        arm.moveToTargetAngleBlocking(100, this::tick);
        arm.moveToTargetExtensionBlocking(Arm.MAX_ARM_EXTENSION - 0.25, this::tick);
        prettyTelem.addLine("arm in place");

        collector.moveWristToBlocking(Collector.UP_POSITION, this::tick, false);
        prettyTelem.addLine("what the fuck 2");

        collector.setGripPosition(0.6);
        sleep(1250);
        collector.moveWristToBlocking(Collector.DOWN_POSITION, this::tick, false);

    }

    public void tick(){
        arm.tick();
        collector.tick();
        prettyTelem.update();
        tickTimer.reset();
    }


    /**
     * Complete delivering a sample (if true) and the AutoTask.
     * @implNote This should follow a unique path depending on DELIVER_SPECIMEN and the AutoTask.
     */
    public void run(){
        //TODO FOR EBEN - finish implementing this

        if (isStopRequested()) return;
        sampleDeliveryPath();




        sleep(10000);
    }

    //todo add methods, each corresponding to a move you want to make in autonomous
}
