package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.apriltag.AprilTagReader;
import org.firstinspires.ftc.teamcode.apriltag.Camera;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Collector;
import org.firstinspires.ftc.teamcode.hardware.drive.DriveBase;
import org.firstinspires.ftc.teamcode.hardware.ScoringElementColor;
import org.firstinspires.ftc.teamcode.hardware.drive.Pose;
import org.firstinspires.ftc.teamcode.hardware.drive.StandardTrackingWheelLocalizer;

import java.util.List;
import java.util.Locale;

/** @noinspection SpellCheckingInspection*/
@Config
@TeleOp(name="1 - Main TeleOp")
public class OpModeCore extends LinearOpMode {

    //<editor-fold desc="Config">
    public static float LOW_POWER_MODIFIER = 0.25f;
    public static float HIGH_POWER_MODIFIER = 0.75f;
    public static float MAX_INCHES_PER_SECOND = 9f;
    public static float MIN_WRIST_VELOCITY = 1;
    //</editor-fold>

    //<editor-fold desc="Fields">
    private static AprilTagReader aprilTagReader;
    private static OpModeCore instance;
    private static Collector collector;
    private static DriveBase driveBase;
    private static Arm arm;
    private static Autopilot autopilot;


    
    private PrettyTelemetry prettyTelem;

    private final Gamepad previousGamepad1 = new Gamepad();
    private final Gamepad previousGamepad2 = new Gamepad();


    private boolean collectorArmed = false;
    ElapsedTime tickTimer, gamepadTimer;
    private List<LynxModule> lynxModules;
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
                "extensionLimitSensor");
        collector = new Collector(
                arm,
                hardwareMap,
                "colorSensor",
                "wristMotor",
                "gripServo"
        );

        autopilot = new Autopilot(driveBase, arm, collector);
        autopilot.setTickRunnable(this::tick);

        aprilTagReader = new AprilTagReader(
                new Camera(
                        hardwareMap,
                        "Webcam 1",
                        new Pose(0, 0, 0)
                )
        );

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

        prettyTelem.addLine("Color Sensor")
                .addData("HSV", this::getHSV)
                .addData("RGB", this::getRGB)
                .addData("Scoring Color", () -> collector.colorSensor.getScoringElementColor());

        prettyTelem.addData("April Tag", () -> aprilTagReader.getDetectionString());
    }

    private String getHSV(){
        float[] hsv = collector.colorSensor.getHSV();
        return String.format(Locale.ENGLISH,"Hue: %.3f Saturation: %.3f Value: %.3f", hsv[0], hsv[1], hsv[2]);
    }

    private String getRGB(){
        NormalizedRGBA rgba = collector.colorSensor.getRGBA();
        return String.format(Locale.ENGLISH,"Red: %.3f Green: %.3f Blue: %.3f", rgba.red, rgba.green, rgba.blue);
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
        checkGamepad();
        checkForScoringElement();
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

    public void checkForScoringElement(){
        if(collectorArmed){
            if(collector.colorSensor.getScoringElementColor() != ScoringElementColor.NONE){
                collector.closeGrip();
            }
        }
    }

    //this might be moved to a seperate class
    private boolean isHighPower = false;
    private boolean manualArm = false;
    public void checkGamepad() {
        //store the current gamepads since this state can change while in a check cycle
        Gamepad gamepad1 = new Gamepad();
        gamepad1.copy(this.gamepad1);
        Gamepad gamepad2 = new Gamepad();
        gamepad2.copy(this.gamepad2);

        if(gamepad1.left_bumper && !previousGamepad1.left_bumper){
            manualArm = !manualArm;
        }

        if(gamepad1.right_bumper && !previousGamepad1.right_bumper){
            refreshLocations();
        }


        //toggle grip on pressing a, if failed to detect if open or closed, default to close.
        if(gamepad1.a){
            if(!previousGamepad1.a) {
                if (!collector.toggleGrip()) {
                    collector.closeGrip();
                }
            }
        }

        //toggle wrist on pressing b, if failed to detect if up or down, default to up.
        if(gamepad1.b && !previousGamepad1.b){
            collector.setWristMode(Collector.WristMode.MOVE_TO_TARGET);
            if(!collector.toggleWrist())
                collector.wristUp();
        }

        if(gamepad1.y && !previousGamepad1.y){
            collectorArmed = !collectorArmed;
        }

        if(gamepad1.x && !previousGamepad1.x) {
            isHighPower = !isHighPower;
            if (isHighPower) {
                driveBase.setPowerFactor(HIGH_POWER_MODIFIER);
            } else {
                driveBase.setPowerFactor(LOW_POWER_MODIFIER);
            }
        }

        if (gamepad1.dpad_left && !previousGamepad1.dpad_left){
            collector.setWristMode(Collector.WristMode.STAY_PERPENDICULAR);
        }

        if(gamepad1.dpad_right && !previousGamepad1.dpad_right)
            collector.setWristMode(Collector.WristMode.FLOAT);

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


        arm.setTargetExtension(
                arm.getTargetExtension() +
                        gamepadTimer.seconds() * MAX_INCHES_PER_SECOND * (-gamepad1.left_trigger + gamepad1.right_trigger)
        );

        gamepadTimer.reset();

        driveBase.moveUsingPower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        //save the last gamepad state to compare again later
        previousGamepad1.copy(gamepad1);
        previousGamepad2.copy(gamepad2);
    }

    public void refreshLocations(){
        collector.setWristMode(Collector.WristMode.SET_POWER);
        collector.setWristPower(0.15);

        while(collector.getWristVelocity() > MIN_WRIST_VELOCITY){
            collector.tick();
            telemetry.update();
        }

        collector.resetPositionAsTop();
        collector.setWristPower(0);
        collector.setWristMode(Collector.WristMode.MOVE_TO_TARGET);
        collector.wristUp();

        arm.setMode(Arm.ArmMode.SET_POWER);
        arm.setAnglePower(-0.3);
        arm.setExtensionPower(-1);

        while(arm.extensionLimitSensor.isPressed() || arm.tiltLimitSensor.isPressed()){
            arm.tick();
            if(arm.tiltLimitSensor.isPressed())
                arm.setAnglePower(0);
            if(arm.extensionLimitSensor.isPressed())
                arm.setExtensionPower(0);
            telemetry.update();
        }

        arm.resetExtension();
        arm.resetAngle();

        arm.setMode(Arm.ArmMode.MOVE_TO_TARGET);
    }
}
