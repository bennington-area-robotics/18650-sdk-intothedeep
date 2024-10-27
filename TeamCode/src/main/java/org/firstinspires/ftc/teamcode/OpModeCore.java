package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Collector;

/** @noinspection SpellCheckingInspection*/
@TeleOp(name="Main TeleOp", group ="Into The Deep")
public class OpModeCore extends LinearOpMode {
    private static OpModeCore instance;
    private static Collector collector;
    private static DriveBase driveBase;

    private final Gamepad previousGamepad1 = new Gamepad();
    private final Gamepad previousGamepad2 = new Gamepad();

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

    public void initialize(){
        instance = this;

        //initialize hardware
        collector = new Collector(hardwareMap, "colorSensor", "wristServo", "gripServo");
        //driveBase = new DriveBase(hardwareMap);

        configureTelemetry();

        //save the current gamepad states to compare against to avoid errors
        previousGamepad1.copy(gamepad1);
        previousGamepad2.copy(gamepad2);
    }

    private void configureTelemetry(){
        telemetry.setAutoClear(false); //disable clearing telemetry after update() is called

        //use suppliers to allow updating values without clearing and re-adding
        telemetry.addData("Detected Color", collector.colorSensor::getScoringElementColor);
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
        telemetry.update();
    }


    //this might be moved to a seperate class
    public void checkGamepad(){
        //toggle grip on pressing a, if failed to detect if open or closed, default to close.
        if(gamepad1.a && !previousGamepad1.a){
            if(!collector.toggleGrip())
                collector.closeGrip();
        }

        //toggle wrist on pressing b, if failed to detect if up or down, default to up.
        if(gamepad1.b && !previousGamepad1.b){
            if(!collector.toggleWrist())
                collector.wristUp();
        }

        //save the gamepad state to compare again later..
        previousGamepad1.copy(gamepad1);
        previousGamepad2.copy(gamepad2);
    }
}
