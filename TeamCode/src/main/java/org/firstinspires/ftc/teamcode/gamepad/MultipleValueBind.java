package org.firstinspires.ftc.teamcode.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.function.Consumer;
import java.util.stream.Collectors;

public class MultipleValueBind extends MonoBind {
    private final List<GamePad.Value> values;
    private final Consumer<List<Float>> valuesConsumer;

    public MultipleValueBind(Consumer<List<Float>> valuesConsumer, GamePad.Value... values){
        this.valuesConsumer = valuesConsumer;
        this.values = Arrays.asList(values);
    }

    public List<Bindable> accept(Gamepad currentGamepad, Gamepad lastGamepad, List<Bindable> blockedBindables){
        if(blockedBindables.stream().anyMatch(values::contains))
            return Collections.emptyList();

        valuesConsumer.accept(values.stream().map(value -> value.get(currentGamepad)).collect(Collectors.toList()));

        return Collections.emptyList();
    }
}
