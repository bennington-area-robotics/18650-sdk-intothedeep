package org.firstinspires.ftc.teamcode.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.function.Consumer;

public class PressValueBind extends MultiBind {
    private final GamePad.Value value;
    private final Consumer<Float> onValue;

    public PressValueBind(GamePad.Press control, GamePad.Value value, Consumer<Float> onValue){
        super(control);
        assert value != null; assert onValue != null;
        this.onValue = onValue;
        this.value = value;
    }

    @Override
    public List<Bindable> accept(Gamepad currentGamepad, Gamepad lastGamepad, List<Bindable> blockedBindables) {
        if(!(blockedBindables.contains(control) || blockedBindables.contains(value)))
            if(control.isPressed(currentGamepad)){
                onValue.accept(value.get(currentGamepad));
                return Collections.singletonList(value);
            }

        return Collections.emptyList();
    }

    @Override
    public List<Bindable> getBlockedBindables() {
        if(bubble)
            return Collections.emptyList();
        else
            return Arrays.asList(control, value);
    }
}