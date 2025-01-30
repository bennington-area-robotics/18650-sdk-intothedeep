package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.apriltag.AprilTagReader;
import org.firstinspires.ftc.teamcode.apriltag.Camera;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Collector;
import org.firstinspires.ftc.teamcode.hardware.drive.DriveBase;
import org.firstinspires.ftc.teamcode.hardware.ScoringElementColor;
import org.firstinspires.ftc.teamcode.hardware.drive.Pose;

import java.util.Locale;

/** @noinspection SpellCheckingInspection*/
@Config
@TeleOp(name="1 - Main TeleOp")
public class OpModeCore extends LinearOpMode {

    public static float LOW_POWER_MODIFIER = 0.25f;
    public static float HIGH_POWER_MODIFIER = 0.75f;

    private static AprilTagReader aprilTagReader;
    private static OpModeCore instance;
    private static Collector collector;
    private static DriveBase driveBase;
    private static Arm arm;
    private static Autopilot autopilot;
    private static TouchSensor touchSensor;

    private final Gamepad previousGamepad1 = new Gamepad();
    private final Gamepad previousGamepad2 = new Gamepad();

    public static int targetPos = 0;

    private boolean collectorArmed = false;
    ElapsedTime tickTimer;

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

    public void initialize(){
        instance = this;



        //initialize hardware
        collector = new Collector(
                hardwareMap,
                "colorSensor",
                "wristMotor",
                "gripServo"
        );
        driveBase = new DriveBase(hardwareMap);
        arm = new Arm(
                hardwareMap,
                "tiltMotorLeft",
                "tiltMotorRight",
                "extensionMotor",
                "touchSensor"
        );
        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");



        autopilot = new Autopilot(driveBase, arm, collector);
        autopilot.setTickRunnable(this::tick);

        configureTelemetry();

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
    }

    private void configureTelemetry(){
        telemetry.setAutoClear(false);
        telemetry.addLine("System Status")
                .addData("Collector Armed?", () -> collectorArmed)
                .addData("Tick Time", () -> Math.round(tickTimer.milliseconds()))
                .addData("Stage", autopilot.findCurrentStage());

        telemetry.addLine("Arm Status")
                .addData("Current Angle", () -> arm.getCachedAngle())
                .addData("Target Angle", () -> arm.getTargetAngle())
                .addData("Current Extension", () -> arm.getCachedExtension())
                .addData("Target Extension", () -> arm.getTargetExtension())
                .addData("Last Angle Power", () -> arm.getLastAnglePower())
                .addData("Last Extension Power", () -> arm.getLastExtensionPower())
                .addData("Touch Sensor Pressed", () -> touchSensor.isPressed());

        telemetry.addLine("Grip")
                .addData("Position", () -> collector.getGripPosition())
                .addData("Open?", () -> collector.isGripOpen())
                .addData("Closed?", () -> collector.isGripClosed());

        telemetry.addLine("Wrist")
                .addData("Position", () -> collector.getWristPosition())
                .addData("Up?", () -> collector.isWristUp())
                .addData("Down?", () -> collector.isWristDown());

        telemetry.addLine("Color Sensor")
                .addData("HSV", this::getHSV)
                .addData("RGB", this::getRGB)
                .addData("Scoring Color", () -> collector.colorSensor.getScoringElementColor());

        telemetry.addData("April Tag", () -> aprilTagReader.getDetectionString());
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
        checkGamepad();
        checkForScoringElement();
        arm.tick();
        collector.tick();
        telemetry.update();
        tickTimer.reset();
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


        //toggle grip on pressing a, if failed to detect if open or closed, default to close.
        if(gamepad1.a){
            if(!previousGamepad1.a) {
                if (!collector.toggleGrip()) {
                    collector.closeGrip();
                }
            }
        }
        if(gamepad1.right_bumper && !previousGamepad1.right_bumper){
            if(!previousGamepad1.a) {
                arm.setTargetAngle(targetPos);
            }
        }

        //toggle wrist on pressing b, if failed to detect if up or down, default to up.
        if(gamepad1.b && !previousGamepad1.b){
            if(!collector.toggleWrist())
                collector.wristUp();
        }

        if(gamepad1.y && !previousGamepad1.y){
            collectorArmed = !collectorArmed;
        }

        if(gamepad1.left_bumper && !previousGamepad1.left_bumper){
            manualArm = !manualArm;
        }

        if(gamepad1.x && !previousGamepad1.x) {
            isHighPower = !isHighPower;
            if (isHighPower) {
                driveBase.setPowerFactor(HIGH_POWER_MODIFIER);
            } else {
                driveBase.setPowerFactor(LOW_POWER_MODIFIER);
            }
        }

        if(gamepad1.dpad_down && !previousGamepad1.dpad_down){
            if(manualArm){
                arm.setTargetAngle(arm.getTargetAngle() - 15);
            }else{
                collector.wristUp();
                arm.collectionPosition();
            }
        }else if(gamepad1.dpad_up && !previousGamepad1.dpad_up){
            if(manualArm){
                arm.setTargetAngle(arm.getTargetAngle() + 15);
            }else {
                if (!arm.setTargetAngle(95))
                    this.gamepad1.rumbleBlips(100);
            }
        }


        arm.setTargetExtension(arm.getTargetExtension() + 0.11 * (-gamepad1.left_trigger + gamepad1.right_trigger));

        driveBase.move(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        //save the last gamepad state to compare again later
        previousGamepad1.copy(gamepad1);
        previousGamepad2.copy(gamepad2);
    }
}
