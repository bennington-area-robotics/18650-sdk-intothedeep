package org.firstinspires.ftc.teamcode.core.implementations;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.PitchWrist;
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
                if (!grip.toggleGrip()) {
                    grip.closeGrip();
                }
            }
        }

        //toggle wrist on pressing b, if failed to detect if up or down, default to up.
        if(gamepad1.b && !previousGamepad1.b){
            pitch.setMode(PitchWrist.Mode.MOVE_TO_TARGET);
            if(!pitch.toggle())
                pitch.pitchUp();
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
            pitch.setMode(PitchWrist.Mode.MOVE_TO_TARGET);
            pitch.pitchTo(-34);
        }

        //dpad down -> if arm is in manual mode move arm down 15 degrees,
        // else bring arm pull the arm all the way in and all the way down
        //dpad up -? if arm is in manual mode move arm up 15 degrees,
        // else bring arm all the way up
        if(gamepad1.dpad_down && !previousGamepad1.dpad_down){
            if(manualArm){
                tilt.setTargetAngle(Math.max(tilt.getTargetAngle() - 15, 0));
            }else{
                pitch.pitchUp();
                arm.telescopeToAsync(0)
                        .thenRun(() -> arm.tiltToAsync(0));
            }
        }else if(gamepad1.dpad_up && !previousGamepad1.dpad_up){
            if(manualArm){
                tilt.setTargetAngle(Math.min(tilt.getTargetAngle() + 15, 100));
            }else {
                if (!tilt.setTargetAngle(100))
                    this.gamepad1.rumbleBlips(100);
            }
        }

        double extensionOffset =  gamepad1.left_trigger - gamepad1.right_trigger * -1;
        if(extensionOffset < 0.02){
            if(!telescoping.setTargetExtension(telescoping.getExtension() + extensionOffset))
                this.gamepad1.rumbleBlips(10);
        }

        //todo add specimen delivery macro

        gamepadTimer.reset();

        driveBase.moveUsingPower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
    }
}
