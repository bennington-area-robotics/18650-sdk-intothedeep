package org.firstinspires.ftc.teamcode.hi;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DriveBase;
import org.firstinspires.ftc.teamcode.PID;

@TeleOp(name= "PIDTest",  group = "Robot")
public class PIDTest extends LinearOpMode {

    private DriveBase driveBase;
    private double[] kValues = {0.01, 0.0001, 0};
    private String[] kNames = {"kP", "kD", "kI"};
    private PIDController pid = new PIDController(kValues[0], kValues[1],kValues[2] );
    private final ElapsedTime runtime = new ElapsedTime();
    private int target = 1000;
    private double power = 0;
    private int index = 0;




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
                        telemetry.addData("kP: ", kValues[0]);
                        telemetry.addData("kD: ", kValues[1]);
                        telemetry.addData("kI: ", kValues[2]);
                        telemetry.update();
                    }
                }

                if (gamepad1.dpad_up){
                    kValues[index]+=0.0001;
                    pid.setK(kValues[index], index);

                    sleep(300);
                }
                if (gamepad1.dpad_down){
                    kValues[index]-=0.0001;
                    pid.setK(kValues[index], index);

                    sleep(300);
                }
                if (gamepad1.dpad_right && index ==2){
                    index = 0;
                    sleep(300);
                } else if (gamepad1.dpad_left && index == 0){
                    index = 2;
                    sleep(300);
                } else if (gamepad1.dpad_right){
                    index++;
                    sleep(300);
                } else if (gamepad1.dpad_left){
                    index--;
                    sleep(300);
                }
                telemetry.addData("Stopped: ", 0);
                telemetry.addData("Editing: ", kNames[index]);
                telemetry.addData("kP: ", kValues[0]);
                telemetry.addData("kD: ", kValues[1]);
                telemetry.addData("kI: ", kValues[2]);
                telemetry.update();
            }
        }

    }




}
