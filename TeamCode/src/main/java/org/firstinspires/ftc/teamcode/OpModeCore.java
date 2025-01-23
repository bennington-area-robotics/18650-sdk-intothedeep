package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Collector;
import org.firstinspires.ftc.teamcode.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.hardware.ScoringElementColor;

/** @noinspection SpellCheckingInspection*/
@Config
@TeleOp(name="Main TeleOp", group ="Into The Deep")
public class OpModeCore extends LinearOpMode {

    public static float LOW_POWER_MODIFIER = 0.25f;
    public static float HIGH_POWER_MODIFIER = 0.75f;

    private static OpModeCore instance;
    private static Collector collector;
    private static DriveBase driveBase;
    private static Arm arm;
    private static Autopilot autopilot;

    private final Gamepad previousGamepad1 = new Gamepad();
    private final Gamepad previousGamepad2 = new Gamepad();

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
                "wristServo",
                "gripServo",
                true
        );
        driveBase = new DriveBase(hardwareMap);
        arm = new Arm(hardwareMap, "tiltMotorLeft", "tiltMotorRight", "extensionMotor", "touchSensor");
        autopilot = new Autopilot(driveBase, arm, collector);
        autopilot.setTickRunnable(this::tick);

        configureTelemetry();

        //save the current gamepad states to compare against to avoid errors
        previousGamepad1.copy(gamepad1);
        previousGamepad2.copy(gamepad2);

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

        telemetry.addData("Arm Extension", arm::getExtension);
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
    public void checkGamepad(){
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

        if(gamepad1.dpad_down){
            if(!arm.setTargetAngle(0))
                this.gamepad1.rumble(100);
            collector.wristUp();
            arm.setTargetExtension(0);
        }else if(gamepad1.dpad_up){
            if(!arm.setTargetAngle(90))
                this.gamepad1.rumbleBlips(100);
        }



        arm.setTargetExtension(arm.getTargetExtension() + 0.11 * (-gamepad1.left_trigger + gamepad1.right_trigger));

        driveBase.move(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        //save the last gamepad state to compare again later
        previousGamepad1.copy(gamepad1);
        previousGamepad2.copy(gamepad2);
    }
}
