package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Main TeleOp", group ="Into The Deep")
public class OpModeCore extends LinearOpMode {
    private static OpModeCore instance;

    public static OpModeCore getInstance(){
        return instance;
    }

    public boolean isActive(){
        return opModeIsActive();
    }

    /*public void initialize(GamepadListener.){
        
    }*/

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
