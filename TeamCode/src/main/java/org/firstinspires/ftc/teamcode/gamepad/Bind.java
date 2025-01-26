package org.firstinspires.ftc.teamcode.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.List;

public interface Bind {
    List<Bindable> accept(Gamepad currentGamepad, Gamepad lastGamepad, List<Bindable> blockedBindables);
}
