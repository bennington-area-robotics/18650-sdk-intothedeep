package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.apriltag.AprilTagReader;
import org.firstinspires.ftc.teamcode.gamepad.GamePad;
import org.firstinspires.ftc.teamcode.gamepad.GamePad.Value;
import org.firstinspires.ftc.teamcode.gamepad.GamePadBuilder;
import org.firstinspires.ftc.teamcode.gamepad.PressBind;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Collector;
import org.firstinspires.ftc.teamcode.hardware.drive.DriveBase;
import org.firstinspires.ftc.teamcode.hardware.ScoringElementColor;

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
    private static GamePad gamePad;

    //so FTC Dashboard can access telemetry
    FtcDashboard dashboard = FtcDashboard.getInstance();
    //Telemetry telemetry = new MultipleTelemetry(super.telemetry, dashboard.getTelemetry());


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
//        aprilTagReader = new AprilTagReader(hardwareMap,
//                new Position(DistanceUnit.INCH, 0, 0, 0, 0),
//                new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0),
//                new Size(640, 480),
//                0
//        );
        collector = new Collector(
                hardwareMap,
                "colorSensor",
                "wristServo",
                "gripServo",
                false
        );
        driveBase = new DriveBase(hardwareMap);
        arm = new Arm(hardwareMap, "tiltMotorLeft", "tiltMotorRight", "extensionMotor", "touchSensor");
        autopilot = new Autopilot(driveBase, arm, collector);
        autopilot.setTickRunnable(this::tick);

        configureTelemetry();
        configGamepad();

        tickTimer = new ElapsedTime();
    }

    private void configureTelemetry(){
        telemetry.setAutoClear(false); //disable clearing telemetry after update() is called
        telemetry.log().setCapacity(100);
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);

        //use suppliers to allow updating values without clearing and re-adding
        //such as: telemetry.addData("Detected Color", collector.colorSensor::getScoringElementColor); DO NOT UNCOMMENT

        telemetry.addData("Collector Armed? ", () -> collectorArmed);
        telemetry.addData("Tick Time ", () -> Math.round(tickTimer.milliseconds()));
        telemetry.addData("Stage", autopilot.findCurrentStage());
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
        gamePad.check();
        checkForScoringElement();
        arm.tick();
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

    private boolean isHighPower = false;

    public void configGamepad(){
        GamePadBuilder builder = new GamePadBuilder(gamepad1);

        builder.addPressBind(GamePad.Press.A, PressBind.Behavior.RISING_EDGE, () -> {
            if (!collector.toggleGrip())
                collector.closeGrip();
        });

        builder.addPressBind(GamePad.Press.B, PressBind.Behavior.RISING_EDGE, () -> {
            if(!collector.toggleWrist())
                collector.wristUp();
        });

        builder.addPressBind(GamePad.Press.Y, PressBind.Behavior.RISING_EDGE, () ->
            collectorArmed = !collectorArmed
        );

        builder.addPressBind(GamePad.Press.X, PressBind.Behavior.RISING_EDGE, () -> {
            isHighPower = !isHighPower;
            if (isHighPower) {
                driveBase.setPowerFactor(HIGH_POWER_MODIFIER);
            } else {
                driveBase.setPowerFactor(LOW_POWER_MODIFIER);
            }
        });

        builder.addPressBind(GamePad.Press.DPAD_DOWN, PressBind.Behavior.RISING_EDGE, () -> {
            collector.wristUp();
            arm.collectionPosition();
        });

        builder.addPressBind(GamePad.Press.DPAD_UP, PressBind.Behavior.RISING_EDGE, () -> {
            if(!arm.setTargetAngle(90))
                this.gamepad1.rumbleBlips(100);
        });

        builder.addMultipleValueBind(values ->
                arm.setTargetExtension(arm.getTargetExtension() + 0.11 * (-values.get(0) + values.get(2)))
        , Value.L_TRIGGER, Value.R_TRIGGER);

        builder.addMultipleValueBind(values ->
                driveBase.move(values.get(0), values.get(1), values.get(2))
        , Value.L_STICK_X, Value.L_STICK_Y, Value.R_STICK_X);

        gamePad = builder.build();

    }
}
