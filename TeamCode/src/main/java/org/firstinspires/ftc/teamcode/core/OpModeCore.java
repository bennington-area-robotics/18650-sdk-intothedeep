package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.GameState;
import org.firstinspires.ftc.teamcode.utilities.PrettyTelemetry;
import org.firstinspires.ftc.teamcode.apriltag.MultiAprilTagReader;
import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.Collector;
import org.firstinspires.ftc.teamcode.components.drive.DriveBase;
import org.firstinspires.ftc.teamcode.utilities.Pose;
import org.firstinspires.ftc.teamcode.components.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.ScoringElementColor;

import java.util.List;
import java.util.Locale;

/** @noinspection SpellCheckingInspection*/
@Config
@TeleOp(name="1 - Main TeleOp")
public class OpModeCore {

    //<editor-fold desc="Config">
    public static float LOW_POWER_MODIFIER = 0.25f;
    public static float HIGH_POWER_MODIFIER = 0.75f;
    public static float MAX_INCHES_PER_SECOND = 9f;
    //</editor-fold>

    //<editor-fold desc="Fields">
    //components
    private static MultiAprilTagReader aprilTagReader;
    private static OpModeCore instance;
    private static Collector collector;
    private static DriveBase driveBase;
    private static Arm arm;
    private static GameState gameState;
    private static TouchSensor touchSensor;

    private final Gamepad previousGamepad1 = new Gamepad();
    private final Gamepad previousGamepad2 = new Gamepad();
    private ElapsedTime tickTimer, gamepadTimer;
    private List<LynxModule> lynxModules;
    private PrettyTelemetry prettyTelem;
    //private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private boolean collectorArmed = false;
    private boolean isHighPower = false;
    private boolean manualArm = false;
    //</editor-fold>

    //<editor-fold desc="Instance Getters">
    public static OpModeCore getInstance(){
        return instance;
    }

    public static PrettyTelemetry getTelemetry(){
        return instance.prettyTelem;
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

    public static GameState getAutopilot(){
        return gameState;
    }
    //</editor-fold>

    public OpModeCore(LinearOpMode base){
        //inject dependencies
        this.prettyTelem = new PrettyTelemetry(base.telemetry);
        Hardware.init(base.hardwareMap);
    }

    public void initialize(){
        instance = this;

        lynxModules = Hardware.getHubs();

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
        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");

        gameState = new GameState(driveBase, arm, collector);
        gameState.setTickRunnable(this::tick);

        aprilTagReader = new MultiAprilTagReader(
                Hardware.getCamera(
                        "Webcam Left",
                        new Pose(-6.5, 2.125, 90)
                ),
                Hardware.getCamera(
                        "Webcam Right",
                        new Pose(6.5, 2.125, -90)
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
        
        prettyTelem.addLine("System Status")
                .addData("Collector Armed?", () -> collectorArmed)
                .addData("Tick Time", () -> Math.round(tickTimer.milliseconds()))
                .addData("Stage", () -> gameState.findCurrentStage())
                .addData("Localization: ", () -> driveBase.getPoseSimple())
        ;
        prettyTelem.addLine("Game State")
                .addData("In Basket Area", () -> gameState.inBasketArea())
                .addData("In Submersible Collection Area", () -> gameState.isInSubmersibleCollectionArea())
                .addData("In Observation Collection Area", () -> gameState.inObservationZoneCollectionArea())
                .addData("In Specimen Delivery Area", () -> gameState.inSpecimenDeliveryArea())
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

        prettyTelem.addLine("Color Sensor")
                .addData("HSV", this::getHSV)
                .addData("RGB", this::getRGB)
                .addData("Scoring Color", () -> collector.colorSensor.getScoringElementColor());

        prettyTelem.addLine("April Tags")
                .addData("Left Camera", () -> aprilTagReader.getFirstPose(0).toString())
                .addData("Right Camera", () -> aprilTagReader.getFirstPose(1).toString());
    }

    private String getHSV(){
        float[] hsv = collector.colorSensor.getHSV();
        return String.format(Locale.ENGLISH,"Hue: %.3f Saturation: %.3f Value: %.3f", hsv[0], hsv[1], hsv[2]);
    }

    private String getRGB(){
        NormalizedRGBA rgba = collector.colorSensor.getNormalizedColors();
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
    public void checkGamepad() {
        //store the current gamepads since this state can change while in a check cycle
        Gamepad gamepad1 = new Gamepad();
        gamepad1.copy(this.gamepad1);
        Gamepad gamepad2 = new Gamepad();
        gamepad2.copy(this.gamepad2);


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

        if(gamepad1.dpad_right && !previousGamepad1.dpad_right) {
            arm.setTargetAngle(30);
            arm.setTargetExtension(9.5);
            collector.setWristMode(Collector.WristMode.MOVE_TO_TARGET);
            collector.wristTo(-34);
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

        gamepadTimer.reset();

        driveBase.moveUsingPower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        //save the last gamepad state to compare again later
        previousGamepad1.copy(gamepad1);
        previousGamepad2.copy(gamepad2);
    }
}
