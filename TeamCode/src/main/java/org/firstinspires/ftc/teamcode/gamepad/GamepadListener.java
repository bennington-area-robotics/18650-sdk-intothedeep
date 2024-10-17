package org.firstinspires.ftc.teamcode.gamepad;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import org.firstinspires.ftc.teamcode.OpModeCore;

import java.util.ArrayList;
import java.util.List;

public class GamepadListener {
    List<GamepadListener> instances = new ArrayList<>();

    public GamepadListener(){
        instances.add(this);
    }

    private void check(){
        gamepad1.
    }

    private static class GamepadListenerThread implements Runnable {
        @Override
        public void run() {
            while(OpModeCore.getInstance().isStarted()){

            }
        }
    }
}
