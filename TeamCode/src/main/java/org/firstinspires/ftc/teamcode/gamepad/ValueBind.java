package org.firstinspires.ftc.teamcode.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Collections;
import java.util.List;
import java.util.function.Consumer;

public class ValueBind extends MonoBind {
    private final GamePad.Value value;
    private final Consumer<Float> onValue;
    private float minimumMagnitude = 0;

    public ValueBind(GamePad.Value value, Consumer<Float> onValue){
        this.value = value;
        this.onValue = onValue;
    }

    /**
     * @param magnitude the minimum magnitude of the value to run the onValue consumer.
     * @return this valuebind.
     */
    public ValueBind minimum(float magnitude){
        this.minimumMagnitude = magnitude;
        return this;
    }

    public List<Bindable> accept(Gamepad currentGamepad, Gamepad lastGamepad, List<Bindable> blockedBindables){
        if(blockedBindables.contains(value))
            return Collections.emptyList();

        onValue.accept(value.get(currentGamepad));
        return Collections.emptyList();
    }
}
