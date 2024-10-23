package org.firstinspires.ftc.teamcode.hi;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DriveBase;
@TeleOp(name= "MotionProfileTest",  group = "Robot")
public class MotionProfileTest extends LinearOpMode {

    private DriveBase driveBase;
    private double maxAcc = 0;
    private int target = 1000;
    private MotionProfileController motionProfile = new MotionProfileController(0.7, maxAcc, target);
    private final ElapsedTime runtime = new ElapsedTime();

    private double power = 0;


    @Override
    public void runOpMode() {
        driveBase = new DriveBase(hardwareMap);
        telemetry.addData("TESTING", "Initialized");
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        runtime.reset();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (gamepad1.x){
                    target = -target;
                    sleep(300);
                }
                if (gamepad1.a) {
                    motionProfile.setTarget(target);
                    while (true) {
                        power = motionProfile.calculate(driveBase.getPosition(), driveBase.getPower());
                        driveBase.setPower(power);
                        if (gamepad1.b) {
                            driveBase.stopMotors();
                            driveBase.reset();
                            break;
                        }
                        telemetry.addData("currently at: ", driveBase.getPosition());
                        telemetry.addData("power: ", driveBase.getPower());
                        telemetry.addData("Max acceleration: ", maxAcc);
                        telemetry.update();
                    }
                }
                if (gamepad1.dpad_up){
                    maxAcc +=0.01;
                    motionProfile.setAcceleration(maxAcc);

                    sleep(300);
                }
                if (gamepad1.dpad_down){
                    maxAcc-=0.01;
                    motionProfile.setAcceleration(maxAcc);

                    sleep(300);
                }
                telemetry.addData("Target: ", target);
                telemetry.addData("Max acceleration: ", maxAcc);
                telemetry.update();


            }
        }
    }
}
