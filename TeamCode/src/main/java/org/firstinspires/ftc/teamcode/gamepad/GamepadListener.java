package org.firstinspires.ftc.teamcode.gamepad;

import android.os.Build;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.teamcode.OpModeCore;

import java.time.Duration;
import java.time.ZonedDateTime;
import java.time.temporal.ChronoUnit;
import java.util.ArrayList;
import java.util.List;

public class GamepadListener {
    private static final GamepadListenerThread thread = new GamepadListenerThread();

    static{
        thread.run();
    }

    private static final List<GamepadListener> instances = new ArrayList<>();
    private final Supplier<Boolean> source;
    private final Runnable onActive;
    private final Duration cooldown;
    private ZonedDateTime lastRun;

    public GamepadListener(Supplier<Boolean> source, Runnable onActive){
        instances.add(this);
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            this.cooldown = Duration.ZERO;
        }else{
            throw new IllegalStateException("Build version was " + Build.VERSION.SDK_INT + " which is not " + Build.VERSION_CODES.O + " or newer, and doesn't support time libraries");
        }
        this.source = source;
        this.onActive = onActive;
        this.lastRun = ZonedDateTime.now().minus(cooldown);
    }

    public GamepadListener(Supplier<Boolean> source, Duration cooldown, Runnable onActive){
        instances.add(this);
        this.cooldown = cooldown;
        this.source = source;
        this.onActive = onActive;
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            this.lastRun = ZonedDateTime.now().minus(cooldown);
        }else{
            throw new IllegalStateException("Build version was " + Build.VERSION.SDK_INT + " which is not " + Build.VERSION_CODES.O + " or newer, and doesn't support time libraries");
        }
    }

    private static void check(){
        for (GamepadListener listener : instances) {
            if(listener.source.get())
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                    if(listener.lastRun.until(ZonedDateTime.now(), ChronoUnit.MILLIS) >= listener.cooldown.get(ChronoUnit.MILLIS)) {
                        listener.lastRun = ZonedDateTime.now();
                        listener.onActive.run();
                    }
                }else{
                    throw new IllegalStateException("Build version was " + Build.VERSION.SDK_INT + " which is not " + Build.VERSION_CODES.O + " or newer, and doesn't support time libraries");
                }
        }
    }

    private static class GamepadListenerThread implements Runnable {
        @Override
        public void run() {
            OpModeCore opMode = OpModeCore.getInstance();
            while(opMode.isStarted() && opMode.opModeIsActive()){
                check();
            }
        }
    }
}
