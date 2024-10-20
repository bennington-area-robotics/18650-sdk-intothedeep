package org.firstinspires.ftc.teamcode.hi;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DriveBase;
import org.firstinspires.ftc.teamcode.PID;

@TeleOp(name= "PIDTest",  group = "Robot")
public class PIDTest extends LinearOpMode {

    DriveBase driveBase;

    double kP = 0.02;
    double kD = 0.0001;
    double kI = 0.01;
    PIDController pid = new PIDController(kP, kI,kD );
    private final ElapsedTime runtime = new ElapsedTime();
    int target = 1000;
    double power = 0;




    @Override
    public void runOpMode(){
        driveBase = new DriveBase(hardwareMap);
        telemetry.addData("TESTING", "Initialized");
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        runtime.reset();
        if (opModeIsActive()) {
            //driveBase.setTargets(target, target, target, target);

            while (opModeIsActive()) {

                if(gamepad1.a) {
                    pid.setSetPoint(target);
                    runtime.reset();
                    while(true) {
                        power = pid.calculate(driveBase.getPosition(), runtime.milliseconds());
                        driveBase.setPower(0.5*power);
                        if (gamepad1.b){
                            driveBase.stopMotors();
                            driveBase.reset();
                            pid.reset();
                            break;
                        }
                        telemetry.addData("currently at: ", driveBase.getPosition());
                        telemetry.addData("power: ", driveBase.getPower());
                        telemetry.addData("kP: ", kP);
                        telemetry.addData("kD: ", kD);
                        telemetry.addData("kI: ", kI);
                        telemetry.update();
                    }
                }

                if (gamepad1.dpad_up){
                    kI+=0.0001;
                    pid.setkI(kI);

                    sleep(300);
                }
                if (gamepad1.dpad_down){
                    kI-=0.0001;
                    pid.setkI(kI);

                    sleep(300);

                }
                telemetry.addData("Stopped: ", 0);
                telemetry.addData("kP: ", kP);
                telemetry.addData("kD: ", kD);
                telemetry.addData("kI: ", kI);
                telemetry.update();
            }
        }

    }




}
