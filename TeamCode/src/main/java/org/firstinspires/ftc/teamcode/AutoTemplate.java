package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.apriltag.AprilTagReader;
import org.firstinspires.ftc.teamcode.apriltag.Camera;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Collector;
import org.firstinspires.ftc.teamcode.hardware.drive.ConfiguredMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.drive.DriveBase;
import org.firstinspires.ftc.teamcode.hardware.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.hardware.drive.Pose;

import java.util.List;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Config
public class AutoTemplate extends LinearOpMode {

    public static boolean manuallyCaching = false;

    private List<LynxModule> lynxModules;
    //TODO FOR EBEN - clean up this code! remove the unnecessary code if its commented, implement the methods I added here
    public static double maxVel = 30, maxAcc = 20, maxAngVel = 2;
    protected TrajectoryVelocityConstraint velocityConstraint = ConfiguredMecanumDrive.getVelocityConstraint(
            maxVel,
            maxAngVel,
            DriveConstants.TRACK_WIDTH);

    protected TrajectoryAccelerationConstraint accelerationConstraint = ConfiguredMecanumDrive.getAccelerationConstraint(
            maxAcc);
    private static ElapsedTime waitTimer = new ElapsedTime();
    protected ElapsedTime tickTimer;
    protected Arm arm;
    protected Collector collector;
    protected PrettyTelemetry prettyTelem;

    protected AprilTagReader aprilTagReader;
    protected DriveBase drive;
    private static Pose2d blueStartPose;
    private static Pose2d redStartPose;

    public static double collectorInitPos = 220;
    public static double armInitAngle = 39;

    //private final Pose2d lastEndPose = startPose;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        run();
    }

    public void wait(double seconds){
        waitTimer.reset();
        setManualCaching();
        while(waitTimer.seconds() <= seconds) {
            tickAll();
        }
    }
    public void setBlueStartPose(double x, double y, double heading){
        blueStartPose = new Pose(x, y, heading).toRR();


    }
    public void setRedStartPose(double x, double y, double heading){
        redStartPose = new Pose(x, y, heading).toRR();

    }
    public Pose2d getRedStartPose(){
        return redStartPose;
    }
    public Pose2d getBlueStartPose(){
        return blueStartPose;
    }

    /**
     * Prepare the robot for autonomous
     * @implNote This should account for where the robot starts, and the position all 'appendages' start in.
     * If there is some initialization movement that needs to happen to allow the bot to start in a position where it will fit in an 18x18, this is when it should happen.
     * Since we get to define the starting position, this is a good time to reset our encoders if necessary.
     */
    private void configureTelemetry(){
        prettyTelem = new PrettyTelemetry(telemetry);
        prettyTelem.addLine ("Errors")
                .addData("Extension", () -> arm.getTargetExtension()-arm.getExtension())
                .addData("Arm Angle", () -> arm.getTargetAngle() - arm.getAngle())
                .addData("Wrist Angle", () -> collector.getWristTarget() - collector.getWristAngle())
        ;
        prettyTelem.addLine("System Status")
                .addData("Localization: ", () -> drive.getPoseSimple())
                .addData("Caching Mode", () -> lynxModules.get(0).getBulkCachingMode())
                .addData("Manually Caching", () -> manuallyCaching)
        ;

        prettyTelem.addLine("Arm Status")
                .addData("Current Angle", () -> arm.getAngle())
                .addData("Target Angle", () -> arm.getTargetAngle())
                .addData("Current Extension", () -> arm.getExtension())
                .addData("Current Extension w/ Encoder", () -> arm.getExtensionEncoderPosition())
                .addData("Target Extension", () -> arm.getTargetExtension())
                .addData("Last Angle Power", () -> arm.getLastAnglePower())
                .addData("kF", () -> arm.upwardKF)
                .addData("Error", () -> arm.getTargetAngle() - arm.getAngle())
                .addData("Last Extension Power", () -> arm.getLastExtensionPower())
                .addData("Tilt Limit Sensor Pressed?", () -> arm.tiltLimitSensor.isPressed())
                .addData("Extension Limit Sensor Pressed?", () -> arm.extensionLimitSensor.isPressed());

        prettyTelem.addLine("Grip")
                .addData("Position", () -> collector.getGripPosition())
                .addData("Open?", () -> collector.isGripOpen())
                .addData("Closed?", () -> collector.isGripClosed());

        prettyTelem.addLine("Wrist")
                .addData("Position", () -> collector.getWristAngle())
                .addData("Angle from ground", () -> collector.getWristAngle() + arm.getAngle())
                .addData("Target", collector::getWristTarget)
                .addData("Velocity", () -> collector.getWristVelocity())
                .addData("Power", () -> collector.getWristPower())
                .addData("Moving with gravity", () -> collector.isWithGravity())
                .addData("Up?", () -> collector.isWristUp())
                .addData("Down?", () -> collector.isWristDown())
                .addData("Rotated?", () -> collector.isWristRotated())
                .addData("Default?", () -> collector.isWristDefault())
                .addData("Limit Sensor Pressed", () -> collector.wristTouchSensor.isPressed());
    }
    public void updateMotorServoCache(){
        if (!manuallyCaching){
            return;
        }
        for(LynxModule module : lynxModules){
            module.clearBulkCache();
        }
    }

    protected void performWithManualCaching(Runnable operation) {
        // Save current caching mode
        LynxModule.BulkCachingMode originalMode = null;
        if (!lynxModules.isEmpty()) {
            originalMode = lynxModules.get(0).getBulkCachingMode();
        }
        assert !lynxModules.isEmpty();

        // Set manual caching
        setManualCaching();

        // Perform the operation
        operation.run();

        // Restore original mode if it was auto
        if (originalMode == LynxModule.BulkCachingMode.AUTO) {
            setAutoCaching();
        }
    }
    public void initialize(){


        lynxModules = hardwareMap.getAll(LynxModule.class);

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
                "wristMotor",
                "gripServo",
                "wristServo",
                "wristLimitSensor");
        tickTimer = new ElapsedTime();
        Arm.upwardKP = 0.055;
        configureTelemetry();
        initializeStartingPosition();


        //TODO FOR EBEN - finish implementing this
    }

    public void initializeStartingPosition(){
        setManualCaching();
        collector.wristToHalfway();
        arm.moveToTargetAngleBlocking(armInitAngle, this::tickInit);
        collector.moveWristToBlocking(collectorInitPos, this::tickInit, true);
        collector.closeGrip();
        arm.setAnglePower(0);
        setAutoCaching();
    }

    protected void setAutoCaching() {
        for (LynxModule module : lynxModules) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        manuallyCaching = false;
    }

    protected void setManualCaching() {
        for (LynxModule module : lynxModules) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        manuallyCaching = true;
    }

    public void resetPosition(){
        arm.collectionPosition();
        collector.wristUp();
    }


    public void tickInit(){

        arm.tick();
        collector.tick();
        updateMotorServoCache();
        tickTimer.reset();
    }
    public void tickAll(){
        arm.tick();
        collector.tick();
        prettyTelem.update();
        tickTimer.reset();
        updateMotorServoCache();
    }
    public void tickArm(){
        updateMotorServoCache();
        prettyTelem.update();
        tickTimer.reset();

    }
    public void tickCollector(){
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
        collector.moveWristToBlocking(0, this::tickAll, false);



        sleep(10000);
    }

    //todo add methods, each corresponding to a move you want to make in autonomous
}
