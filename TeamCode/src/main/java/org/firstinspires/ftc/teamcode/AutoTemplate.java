package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.config.Config;
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
public class AutoTemplate extends LinearOpMode {

    //TODO FOR EBEN - clean up this code! remove the unnecessary code if its commented, implement the methods I added here

    public static double blueStartX = -23.5;
    public static double blueStartY = 63;
    public static double blueStartAng = 90;

    public static double redStartX = 0;
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
                .addData("Up?", () -> collector.isWristUp())
                .addData("Down?", () -> collector.isWristDown());


        prettyTelem.addData("April Tag", () -> aprilTagReader.getFirstPose().toString());
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
                "extensionLimitSensor"
                , this);
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
        initializeStartingPosition();
        configureTelemetry();

        //TODO FOR EBEN - finish implementing this
    }

    public void initializeStartingPosition(){
        arm.moveToTargetAngleBlocking(36, this::tick);
        collector.moveWristToBlocking(240, this::tick, true);
        collector.closeGrip();
        arm.setAnglePower(0);
    }


    public void tick(){

        arm.tick();
        collector.tick();
        //prettyTelem.update();
        tickTimer.reset();
    }


    /**
     * Complete delivering a sample (if true) and the AutoTask.
     * @implNote This should follow a unique path depending on DELIVER_SPECIMEN and the AutoTask.
     */
    public void run(){
        //TODO FOR EBEN - finish implementing this

        if (isStopRequested()) return;
        collector.moveWristToBlocking(0, this::tick, false);



        sleep(10000);
    }

    //todo add methods, each corresponding to a move you want to make in autonomous
}
