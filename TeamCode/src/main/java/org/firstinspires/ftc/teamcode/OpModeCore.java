package org.firstinspires.ftc.teamcode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gamepad.GamepadListener;

import java.time.Duration;
import java.time.temporal.ChronoUnit;
import java.util.ArrayList;
import java.util.List;

/** @noinspection SpellCheckingInspection*/
@TeleOp(name="Main TeleOp", group ="Into The Deep")
public class OpModeCore extends LinearOpMode {
    private static OpModeCore instance;
    List<GamepadListener> listeners = new ArrayList<>();

    public static OpModeCore getInstance(){
        return instance;
    }

    @RequiresApi(api = Build.VERSION_CODES.O)
    public void initialize(){
        instance = this;
        listeners.add(
                new GamepadListener(() -> gamepad1.a, () -> {
                    telemetry.addData("hi", "hii");
                    telemetry.update();
                })
        );
        listeners.add(
                new GamepadListener(() -> gamepad1.b, Duration.of(1500, ChronoUnit.MILLIS),  () -> {
                    telemetry.addData("bb", "bbbbbbbbbbbbbbbbbbbbb");
                    telemetry.update();
                })
        );
    }

    @Override
    public void runOpMode() {

    }
}
