package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name= "PIDTest",  group = "Robot")
public class PIDTest extends LinearOpMode {

    DriveBase driveBase;
    PID pid = new PID();
    private final ElapsedTime runtime = new ElapsedTime();
    int target = 300;


    @Override
    public void runOpMode(){
        driveBase = new DriveBase(hardwareMap);
        telemetry.addData("TESTING", "Initialized");
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        runtime.reset();
        if(opModeIsActive()){
            pid.getPowerLvl(target, driveBase.getPosition());
            driveBase.setPower(0.5,0.5,0.5,0.5);
            sleep(500);
            driveBase.stopMotors();
        }

    }




}
