package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.GameState;
import org.firstinspires.ftc.teamcode.utilities.PersistentStorage;
import org.firstinspires.ftc.teamcode.utilities.PrettyTelemetry;
import org.firstinspires.ftc.teamcode.vision.MultiAprilTagReader;
import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.Collector;
import org.firstinspires.ftc.teamcode.components.DriveBase;
import org.firstinspires.ftc.teamcode.utilities.Pose;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.hardware.Hardware;

import java.util.List;
import java.util.Locale;

@Config
public abstract class OpModeCore extends LinearOpMode{

    //<editor-fold desc="Fields">
    //components
    protected static MultiAprilTagReader aprilTagReader;
    protected static OpModeCore instance;
    protected static Collector collector;
    protected static DriveBase driveBase;
    protected static Arm arm;
    protected static GameState gameState;
    protected ElapsedTime tickTimer, gamepadTimer;
    protected List<LynxModule> lynxModules;
    protected PrettyTelemetry prettyTelem;
    //protected final FtcDashboard dashboard = FtcDashboard.getInstance();

    protected boolean collectorArmed = false;
    protected boolean isHighPower = false;
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

    protected void initialize(){
        instance = this;

        Hardware.init(hardwareMap);
        PersistentStorage.init(hardwareMap);

        this.prettyTelem = new PrettyTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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

        gameState = new GameState(driveBase, arm, collector);

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

        tickTimer = new ElapsedTime();
        gamepadTimer = new ElapsedTime();

        StandardTrackingWheelLocalizer.reverseEncoders();

        // always configure telemetry last
        configureTelemetry();
    }

    protected void configureTelemetry(){
        
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
                .addData("Tilt Power", () -> arm.getAnglePower())
                .addData("Extension Power", () -> arm.getLastExtensionPower())
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

    protected String getHSV(){
        float[] hsv = collector.colorSensor.getHSV();
        return String.format(Locale.ENGLISH,"Hue: %.3f Saturation: %.3f Value: %.3f", hsv[0], hsv[1], hsv[2]);
    }

    protected String getRGB(){
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
        Hardware.invalidateCaches();
        arm.tick();
        collector.tick();
        prettyTelem.update();
        tickTimer.reset();
    }
}
