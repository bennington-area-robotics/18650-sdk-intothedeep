package org.firstinspires.ftc.teamcode.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;

public class GamePadBuilder {
    private final Gamepad baseGamepad;

    private final List<MonoBind> binds = new ArrayList<>();
    private final List<MultiBind> multiBinds = new ArrayList<>();

    /**
     * @param baseGamepad the base gamepad where presses should be read from.
     */
    public GamePadBuilder(Gamepad baseGamepad){
        assert baseGamepad != null;
        this.baseGamepad = baseGamepad;
    }

    public GamePadBuilder addValueBind(GamePad.Value value, Consumer<Float> valueConsumer){
        binds.add(new ValueBind(value, valueConsumer));
        return this;
    }

    public GamePadBuilder addPressBind(GamePad.Press press, PressBind.Behavior behavior, Runnable onPress){
        binds.add(new PressBind(press, behavior, onPress));
        return this;
    }

    public GamePadBuilder addMultipleValueBind(Consumer<List<Float>> valuesConsumer, GamePad.Value... values){
        binds.add(new MultipleValueBind(valuesConsumer, values));
        return this;
    }

    public GamePadBuilder addDoublePressBind(GamePad.Press controlPress, GamePad.Press press, DoublePressBind.Behavior behavior, Runnable onPress){
        multiBinds.add(new DoublePressBind(controlPress, press, behavior, onPress));
        return this;
    }

    public GamePadBuilder addPressValueBind(GamePad.Press control, GamePad.Value value, Consumer<Float> valueConsumer){
        multiBinds.add(new PressValueBind(control, value, valueConsumer));
        return this;
    }

    public GamePad build(){
        return new GamePad(baseGamepad, binds, multiBinds);
    }
}
