package org.firstinspires.ftc.teamcode.core.implementations;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.Collector;
import org.firstinspires.ftc.teamcode.core.TeleOpCore;

@TeleOp(name = "1 - Main TeleOp")
public class MainTeleOp extends TeleOpCore {
    private final boolean manualArm = false;
    private boolean isHighPower = false;
    protected ElapsedTime gamepadTimer;

    //<editor-fold desc="Config">
    public static float LOW_POWER_MODIFIER = 0.25f;
    public static float HIGH_POWER_MODIFIER = 0.75f;
    public static float MAX_INCHES_PER_SECOND = 9f;
    //</editor-fold>


    @Override
    protected void initialize(){
        super.initialize();
        gamepadTimer = new ElapsedTime();
    }

    @Override
    protected void checkGamepad(Gamepad gamepad1, Gamepad gamepad2, Gamepad lastGamepad1, Gamepad lastGamepad2) {
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

        if(gamepad1.x && !previousGamepad1.x) {
            isHighPower = !isHighPower;
            if (isHighPower) {
                driveBase.setPowerFactor(HIGH_POWER_MODIFIER);
            } else {
                driveBase.setPowerFactor(LOW_POWER_MODIFIER);
            }
        }

        if(gamepad1.dpad_right && !previousGamepad1.dpad_right) {
            tilt.setTargetAngle(30);
            telescoping.setTargetExtension(9.5);
            collector.setWristMode(Collector.WristMode.MOVE_TO_TARGET);
            collector.wristTo(-34);
        }

        //dpad down -> if arm is in manual mode move arm down 15 degrees,
        // else bring arm pull the arm all the way in and all the way down
        //dpad up -? if arm is in manual mode move arm up 15 degrees,
        // else bring arm all the way up
        if(gamepad1.dpad_down && !previousGamepad1.dpad_down){
            if(manualArm){
                tilt.setTargetAngle(Math.max(tilt.getTargetAngle() - 15, 0));
            }else{
                collector.wristUp();
                arm.telescopeToAsync(0)
                        .thenRun(() -> arm.tiltTo(0));
            }
        }else if(gamepad1.dpad_up && !previousGamepad1.dpad_up){
            if(manualArm){
                tilt.setTargetAngle(Math.min(tilt.getTargetAngle() + 15, 100));
            }else {
                if (!tilt.setTargetAngle(100))
                    this.gamepad1.rumbleBlips(100);
            }
        }

        gamepadTimer.reset();

        driveBase.moveUsingPower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
    }
}
