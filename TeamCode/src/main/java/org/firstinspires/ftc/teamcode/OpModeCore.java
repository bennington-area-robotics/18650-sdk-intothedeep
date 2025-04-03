package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.apriltag.Camera;
import org.firstinspires.ftc.teamcode.apriltag.MultiAprilTagReader;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Collector;
import org.firstinspires.ftc.teamcode.hardware.drive.DriveBase;
import org.firstinspires.ftc.teamcode.hardware.ScoringElementColor;
import org.firstinspires.ftc.teamcode.hardware.drive.Pose;
import org.firstinspires.ftc.teamcode.hardware.drive.StandardTrackingWheelLocalizer;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/** @noinspection SpellCheckingInspection*/
@Config
@TeleOp(name="1 - Main TeleOp")
public class OpModeCore extends LinearOpMode {

    public static double sampleDeliveryArmAngle = 90;
    public static double collectionArmAngle = 35, collectionArmExtension = 0, collectionCollectorAngle = -25;
    public static double deliveryArmAngle = 55, deliveryArmExtension = 6.5, deliveryCollectorAngle = 40;
    public static int posVariable = 40;
    public static int collectionPosVariable = -20;
    public static double armVariable = 50;
    public static double extensionPosVariable = 8;
    //<editor-fold desc="Config">
    public static float LOW_POWER_MODIFIER = 0.75f;
    public static float HIGH_POWER_MODIFIER = 0.75f;
    public static float MAX_INCHES_PER_SECOND = 9f;
    public static float MIN_WRIST_VELOCITY = 8;
    //</editor-fold>

    public static boolean resettingWrist = false;
    public static boolean manualArmPowerMode = false;
    public static boolean startAfterAscent = true;
    public static double squareUpAngle = 90;
    //<editor-fold desc="Fields">
    //components
    private static MultiAprilTagReader aprilTagReader;
    private static OpModeCore instance;
    private static Collector collector;
    private static DriveBase driveBase;
    private static Arm arm;
    private static Autopilot autopilot;

    private final Gamepad previousGamepad1 = new Gamepad();
    private final Gamepad previousGamepad2 = new Gamepad();
    private ElapsedTime tickTimer, gamepadTimer;
    private List<LynxModule> lynxModules;
    private PrettyTelemetry prettyTelem;
    //private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private boolean collectorArmed = false;
    private boolean isHighPower = true;
    private boolean manualArm = false;
    public static boolean dualControllers = true;
    public static boolean testingPID = false;

    public static DriveMode driveMode = DriveMode.DIRECTIVE;

    public enum DriveMode {
        DIRECTIVE, PURE_POWER, ACCELERATION
    }

    private String testValue = "UNSET";
    //</editor-fold>

    //<editor-fold desc="Instance Getters">
    public static OpModeCore getInstance(){
        return instance;
    }

    public static Telemetry getTelemetry(){
        return instance.telemetry;
    }

    public static Collector getCollector(){
        return collector;
    }

    public static DriveBase getDriveBase(){
        return driveBase;
    }

    public static Arm getArm(){
        return arm;
    }

    public static Autopilot getAutopilot(){
        return autopilot;
    }
    //</editor-fold>

    public void initialize(){
        instance = this;

        lynxModules = hardwareMap.getAll(LynxModule.class);

        for(LynxModule module : lynxModules){
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        //initialize hardware
        driveBase = new DriveBase(hardwareMap);

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
                "wristMotor",
                "gripServo",
                "wristServo",
                "wristLimitSensor"
        );

        autopilot = new Autopilot(driveBase, arm, collector);
        autopilot.setTickRunnable(this::tick);

        if(startAfterAscent){
            arm.resetAngleAfterAscent();
        }
        manualArmPowerMode = false;
        //save the current gamepad states to compare against to avoid errors
        previousGamepad1.copy(gamepad1);
        previousGamepad2.copy(gamepad2);

        tickTimer = new ElapsedTime();
        gamepadTimer = new ElapsedTime();

        StandardTrackingWheelLocalizer.reverseEncoders();


        // always configure telemetry last
        configureTelemetry();
    }

    private void configureTelemetry(){
        prettyTelem = new PrettyTelemetry(telemetry);
        
        prettyTelem.addLine("System Status")
                .addData("Collector Armed?", () -> collectorArmed)
                .addData("Tick Time", () -> Math.round(tickTimer.milliseconds()))
                .addData("Stage", () -> autopilot.findCurrentStage())
                .addData("Localization: ", () -> driveBase.getPoseSimple())
                .addData("Combined Angle", () -> arm.getAngle() + collector.getWristAngle())
                .addData("Arm using manual power", () -> manualArmPowerMode)
        ;
        prettyTelem.addLine("Game State")
                .addData("In Basket Area", () -> autopilot.inBasketArea())
                .addData("In Submersible Collection Area", () -> autopilot.isInSubmersibleCollectionArea())
                .addData("In Observation Collection Area", () -> autopilot.inObservationZoneCollectionArea())
                .addData("In Specimen Delivery Area", () -> autopilot.inSpecimenDeliveryArea())
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

        /*prettyTelem.addLine("Color Sensor")
                .addData("HSV", this::getHSV)
                .addData("RGB", this::getRGB)
                //.addData("Scoring Color", () -> collector.colorSensor.getScoringElementColor());
        */
        prettyTelem.addLine("Check Both Gamepads");
    }


    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        while(opModeIsActive()){
            tick();
        }
    }

    public void tick(){
        updateMotorServoCache();
        driveBase.updatePoseEstimate();

        if(testingPID){
            pidTestGamepad();
        }
        else if(dualControllers){
            checkBothGamepads();
        } else {
            checkGamepad();
        }

        //checkBothGamepads();
        //checkForScoringElement();
        driveBase.update();
        arm.tick();
        collector.tick();
        prettyTelem.update();
        tickTimer.reset();
    }

    public void updateMotorServoCache(){
        for(LynxModule module : lynxModules){
            module.clearBulkCache();
        }
    }

    public void checkBothGamepads(){
        //store the current gamepads since this state can change while in a check cycle
        Gamepad gamepad1 = new Gamepad();
        gamepad1.copy(this.gamepad1);
        Gamepad gamepad2 = new Gamepad();
        gamepad2.copy(this.gamepad2);

        /*if(gamepad1.left_bumper && !previousGamepad1.left_bumper){
            manualArm = !manualArm;
        }*/



        //toggle grip on pressing a, if failed to detect if open or closed, default to close.
        if(gamepad1.a && !previousGamepad1.a){
            if (!collector.toggleGrip()) {
                collector.closeGrip();
            }

        }

        //toggle wrist on pressing b, if failed to detect if up or down, default to up.
        if(gamepad1.b && !previousGamepad1.b){
            collector.setWristMode(Collector.WristMode.MOVE_TO_TARGET);
            if(!collector.toggleWrist())
                collector.wristUp();
        }

        if(gamepad1.y && !previousGamepad1.y){
            if(!collector.toggleWristServo())
                collector.wristToDefaultPosition();
            //collectorArmed = !collectorArmed;
        }

        if(gamepad2.x && !previousGamepad2.x) {
            isHighPower = !isHighPower;
            if (isHighPower) {
                driveBase.setPowerFactor(HIGH_POWER_MODIFIER);
                driveBase.setLowPowerMode(false);
            } else {
                driveBase.setPowerFactor(LOW_POWER_MODIFIER);
                driveBase.setLowPowerMode(true);
            }
        }

        if (gamepad1.dpad_left && !previousGamepad1.dpad_left){
            arm.setTargetAngle(deliveryArmAngle);
            arm.setTargetExtension(deliveryArmExtension);
            collector.wristToDefaultPosition();
            collector.setWristMode(Collector.WristMode.MOVE_TO_TARGET);
            collector.wristTo(deliveryCollectorAngle);
            //collector.setWristMode(Collector.WristMode.STAY_PERPENDICULAR);
        }

        if(gamepad1.dpad_right && !previousGamepad1.dpad_right) {
            arm.setTargetAngle(collectionArmAngle);
            arm.setTargetExtension(collectionArmExtension);
            collector.wristToDefaultPosition();
            collector.setWristMode(Collector.WristMode.MOVE_TO_TARGET);
            collector.wristTo(collectionCollectorAngle);
        }
        if(gamepad1.right_bumper && !previousGamepad1.right_bumper){
            ElapsedTime waitTimer = new ElapsedTime();
            collector.setWristMode(Collector.WristMode.SET_POWER);
            collector.setWristPower(0.4);
            resettingWrist = true;
            /*waitTimer.reset();
            while(!collector.wristTouchSensor.isPressed() && waitTimer.seconds() < 2){
                tick();
            }
            waitTimer.reset();
            while(waitTimer.seconds() < 0.3){
                tick();
            }
            collector.wristUp();*/
        }
        if(resettingWrist && collector.wristTouchSensor.isPressed()){
            collector.setWristMode(Collector.WristMode.MOVE_TO_TARGET);
            collector.wristTo(90);
            resettingWrist = false;
        }

        if(gamepad1.dpad_down && !previousGamepad1.dpad_down){
            if(manualArm){
                arm.setTargetAngle(Math.max(arm.getTargetAngle() - 15, 0));
            }else{
                collector.setWristMode(Collector.WristMode.MOVE_TO_TARGET);
                collector.wristUp();
                arm.collectionPosition();
            }
        }else if(gamepad1.dpad_up && !previousGamepad1.dpad_up){
            if(manualArm){
                arm.setTargetAngle(Math.min(arm.getTargetAngle() + 15, 100));
            }else {
                if (!arm.setTargetAngle(100))
                    this.gamepad1.rumbleBlips(100);
            }
        }



        if(Math.abs(-gamepad1.left_trigger + gamepad1.right_trigger) > 0.1){
            arm.killMacro();
            arm.setExtensionPower(-gamepad1.left_trigger + gamepad1.right_trigger);
        }else if (!arm.isRunningMacro()){
            if(arm.getExtensionMode() != Arm.ExtensionMode.MOVE_TO_TARGET) {
                arm.setExtensionPower(0);
                arm.setTargetExtension(arm.getExtension());
            }
        }
        if(gamepad1.left_stick_button && gamepad1.right_stick_button && !previousGamepad1.right_stick_button){
            arm.setTargetAngle(100);
            collector.setWristMode(Collector.WristMode.MOVE_TO_TARGET);
            collector.wristTo(-40);
        }
        if(gamepad2.left_bumper && !previousGamepad2.left_bumper){
            manualArmPowerMode = !manualArmPowerMode;
        }
        if(manualArmPowerMode){
            arm.setAngleMode(Arm.AngleMode.SET_POWER);
            arm.setAnglePower(-Math.abs(gamepad2.right_stick_y));
        }
        if(!manualArmPowerMode){
            arm.setAngleMode(Arm.AngleMode.MOVE_TO_TARGET);
        }

        gamepadTimer.reset();

        switch (driveMode){
            case DIRECTIVE:
                driveBase.moveUsingRR(gamepad2.left_stick_x, gamepad2.left_stick_y, gamepad2.right_stick_x);
                break;
            case PURE_POWER:
                driveBase.moveUsingPower(gamepad2.left_stick_x, gamepad2.left_stick_y, gamepad2.right_stick_x);
                break;
            case ACCELERATION:
                driveBase.moveWithAcceleration(gamepad2.left_stick_x, gamepad2.left_stick_y, gamepad2.right_stick_x);
        }

        //save the last gamepad state to compare again later
        previousGamepad1.copy(gamepad1);
        previousGamepad2.copy(gamepad2);

    }

    public void pidTestGamepad(){
        Gamepad gamepad1 = new Gamepad();
        gamepad1.copy(this.gamepad1);

        if (gamepad1.dpad_left && !previousGamepad1.dpad_left){
            arm.setTargetAngle(armVariable);
            //collector.setWristMode(Collector.WristMode.STAY_PERPENDICULAR);
        }

        if(gamepad1.dpad_right && !previousGamepad1.dpad_right) {
            arm.setTargetAngle(30);
        }

        if(gamepad1.right_bumper && !previousGamepad1.right_bumper){
            collector.setWristMode(Collector.WristMode.MOVE_TO_TARGET);
            collector.wristTo(collectionPosVariable);
        }
        if (gamepad1.left_bumper && !previousGamepad1.left_bumper){
            arm.setTargetExtension(extensionPosVariable);
        }
        if(gamepad1.dpad_down && !previousGamepad1.dpad_down){
            if(manualArm){
                arm.setTargetAngle(Math.max(arm.getTargetAngle() - 15, 0));
            }else{
                collector.wristUp();
                arm.collectionPosition();
            }
        }else if(gamepad1.dpad_up && !previousGamepad1.dpad_up){
            if(manualArm){
                arm.setTargetAngle(Math.min(arm.getTargetAngle() + 15, 100));
            }else {
                if (!arm.setTargetAngle(100))
                    this.gamepad1.rumbleBlips(100);
            }
        }
        if(gamepad1.b && !previousGamepad1.b){
            collector.setWristMode(Collector.WristMode.MOVE_TO_TARGET);
            if(!collector.toggleWrist())
                collector.wristUp();
        }
        gamepadTimer.reset();
        previousGamepad1.copy(gamepad1);
    }


    //this might be moved to a seperate class
    public void checkGamepad() {
        //store the current gamepads since this state can change while in a check cycle
        Gamepad gamepad1 = new Gamepad();
        gamepad1.copy(this.gamepad1);
        Gamepad gamepad2 = new Gamepad();
        gamepad2.copy(this.gamepad2);




        //toggle grip on pressing a, if failed to detect if open or closed, default to close.
        if(gamepad1.a && !previousGamepad1.a){
            if (!collector.toggleGrip()) {
                collector.closeGrip();
            }

        }

        //toggle wrist on pressing b, if failed to detect if up or down, default to up.
        if(gamepad1.b && !previousGamepad1.b){
            collector.setWristMode(Collector.WristMode.MOVE_TO_TARGET);
            if(!collector.toggleWrist())
                collector.wristUp();
        }

        if(gamepad1.y && !previousGamepad1.y){
            if(!collector.toggleWristServo())
                collector.wristToDefaultPosition();
            //collectorArmed = !collectorArmed;
        }

        if(gamepad1.x && !previousGamepad1.x) {
            isHighPower = !isHighPower;
            if (isHighPower) {
                driveBase.setPowerFactor(HIGH_POWER_MODIFIER);
                driveBase.setLowPowerMode(false);
            } else {
                driveBase.setPowerFactor(LOW_POWER_MODIFIER);
                driveBase.setLowPowerMode(true);
            }
        }

        if (gamepad1.dpad_left && !previousGamepad1.dpad_left){
            arm.setTargetAngle(deliveryArmAngle);
            arm.setTargetExtension(deliveryArmExtension);
            collector.wristToDefaultPosition();
            collector.setWristMode(Collector.WristMode.MOVE_TO_TARGET);
            collector.wristTo(deliveryCollectorAngle);
            //collector.setWristMode(Collector.WristMode.STAY_PERPENDICULAR);
        }

        if(gamepad1.dpad_right && !previousGamepad1.dpad_right) {
            arm.setTargetAngle(collectionArmAngle);
            arm.setTargetExtension(collectionArmExtension);
            collector.wristToDefaultPosition();
            collector.setWristMode(Collector.WristMode.MOVE_TO_TARGET);
            collector.wristTo(collectionCollectorAngle);
        }

        if(gamepad1.right_bumper && !previousGamepad1.right_bumper){
            ElapsedTime waitTimer = new ElapsedTime();
            collector.setWristMode(Collector.WristMode.SET_POWER);
            collector.setWristPower(0.4);
            resettingWrist = true;
            /*waitTimer.reset();
            while(!collector.wristTouchSensor.isPressed() && waitTimer.seconds() < 2){
                tick();
            }
            waitTimer.reset();
            while(waitTimer.seconds() < 0.3){
                tick();
            }
            collector.wristUp();*/
        }
        if(resettingWrist && collector.wristTouchSensor.isPressed()){
            collector.setWristMode(Collector.WristMode.MOVE_TO_TARGET);
            collector.wristTo(90);
            resettingWrist = false;
        }

        if(gamepad1.dpad_down && !previousGamepad1.dpad_down){
            if(manualArm){
                arm.setTargetAngle(Math.max(arm.getTargetAngle() - 15, 0));
            }else{
                collector.setWristMode(Collector.WristMode.MOVE_TO_TARGET);
                collector.wristUp();
                arm.collectionPosition();
            }
        }else if(gamepad1.dpad_up && !previousGamepad1.dpad_up){
            if(manualArm){
                arm.setTargetAngle(Math.min(arm.getTargetAngle() + 15, 100));
            }else {
                if (!arm.setTargetAngle(sampleDeliveryArmAngle))
                    this.gamepad1.rumbleBlips(100);
            }
        }

        if(gamepad1.left_stick_button && gamepad1.right_stick_button && !previousGamepad1.right_stick_button){
            arm.setTargetAngle(100);
            collector.setWristMode(Collector.WristMode.MOVE_TO_TARGET);
            collector.wristTo(-40);
        }
        if(gamepad1.left_bumper && !previousGamepad1.left_bumper){
            manualArmPowerMode = !manualArmPowerMode;
        }

        if(manualArmPowerMode){
            arm.setAngleMode(Arm.AngleMode.SET_POWER);
            arm.setAnglePower(-Math.abs(gamepad1.right_stick_y));
        }

        if(Math.abs(-gamepad1.left_trigger + gamepad1.right_trigger) > 0.1){
            arm.killMacro();
            arm.setExtensionPower(-gamepad1.left_trigger + gamepad1.right_trigger);
        }else if (!arm.isRunningMacro()){
            if(arm.getExtensionMode() != Arm.ExtensionMode.MOVE_TO_TARGET) {
                arm.setExtensionPower(0);
                arm.setTargetExtension(arm.getExtension());
            }
        }

        gamepadTimer.reset();

        //driveBase.moveUsingPower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        driveBase.moveWithAcceleration(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        //save the last gamepad state to compare again later
        previousGamepad1.copy(gamepad1);
        previousGamepad2.copy(gamepad2);
    }
    protected void setAutoCaching() {
        for (LynxModule module : lynxModules) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    protected void setManualCaching() {
        for (LynxModule module : lynxModules) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }


    public void refreshLocations(){
        collector.setWristMode(Collector.WristMode.SET_POWER);
        collector.setWristPower(0.4);

        testValue = "refresh locations!";

        ElapsedTime timer = new ElapsedTime();
        while(timer.milliseconds() < 1250){
            collector.tick();
            testValue = "Collector Moving with velocity " + collector.getWristVelocity();
            prettyTelem.update();
        }

        collector.resetPositionAsTop();

        testValue = "Collector pos reset";
        collector.setWristPower(0);
        collector.setWristMode(Collector.WristMode.MOVE_TO_TARGET);
        collector.moveWristToBlocking(90, this::tick, false);

        arm.setAngleMode(Arm.AngleMode.SET_POWER);
        arm.setAnglePower(-0.2);
        arm.setExtensionPower(-0.5);

        while((!arm.extensionLimitSensor.isPressed() || !arm.tiltLimitSensor.isPressed()) && opModeIsActive()){
            arm.tick();
            if(arm.tiltLimitSensor.isPressed())
                arm.setAnglePower(0);
            if(arm.extensionLimitSensor.isPressed())
                arm.setExtensionPower(0);
            testValue = "Moving arm";
            prettyTelem.update();
        }

        testValue = "Moved arm";
        prettyTelem.update();

        arm.resetExtension();
        arm.resetAngle();

        arm.setAngleMode(Arm.AngleMode.MOVE_TO_TARGET);
    }
}
