package org.firstinspires.ftc.teamcode.core.implementations;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.core.TeleOpCore;
import org.firstinspires.ftc.teamcode.utilities.PersistentStorage;

@Config
@TeleOp(name="Arm Tuner")
public class ArmTuner extends TeleOpCore {
    public static double targetAngle = 0;
    public static double targetExtension = 0;
    private int storedIndex = 0;

    /**
     * Check for button updates on all controllers.
     *
     * @param gamepad1     the current state of gamepad1.
     * @param gamepad2     the current state of gamepad2.
     * @param lastGamepad1 the last state of gamepad1.
     * @param lastGamepad2 the last state of gamepad2.
     */
    @Override
    protected void checkGamepad(Gamepad gamepad1, Gamepad gamepad2, Gamepad lastGamepad1, Gamepad lastGamepad2) {
        if(gamepad1.dpad_up && !lastGamepad1.dpad_up){
            targetAngle = 90;
        }

        if (gamepad1.dpad_down && !lastGamepad1.dpad_down){
            targetAngle = 0;
        }

        if (gamepad1.dpad_right && !lastGamepad1.dpad_right){
            targetAngle = 45;
        }

        arm.setTargetAngle(targetAngle);

        if(gamepad1.x && !lastGamepad1.x){
            storedIndex = PersistentStorage.getInt("temp1821", 0);
            PersistentStorage.saveInt("temp1821", storedIndex + 1);
        }

        if(gamepad1.y && !lastGamepad1.y){
            PersistentStorage.clear();
        }
    }

    @Override
    protected void configureTelemetry() {
        super.configureTelemetry();
        prettyTelem.addDataToDashboard("Arm Angle", arm::getAngle);
        prettyTelem.addDataToDashboard("temp1821", () -> storedIndex);
    }
}
