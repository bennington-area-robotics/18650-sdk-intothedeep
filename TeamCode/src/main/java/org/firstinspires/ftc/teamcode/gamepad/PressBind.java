package org.firstinspires.ftc.teamcode.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Collections;
import java.util.List;

public class PressBind extends MonoBind {
    private final GamePad.Press press;
    private final Behavior behavior;
    private final Runnable onPress;

    public PressBind(GamePad.Press press, Behavior behavior, Runnable onPress){
        this.press = press;
        this.behavior = behavior;
        this.onPress = onPress;
    }

    @Override
    public List<Bindable> accept(Gamepad currentGamepad, Gamepad lastGamepad, List<Bindable> blockedBindables) {
        if(blockedBindables.contains(press))
            return Collections.emptyList();

        switch (behavior) {
            case WHILE_HELD:
                if(press.isPressed(currentGamepad)){
                    onPress.run();
                }
                break;
            case RISING_EDGE:
                if(press.isPressed(currentGamepad) && !press.isPressed(lastGamepad)){
                    onPress.run();
                }
                break;
            case FALLING_EDGE:
                if(!press.isPressed(currentGamepad) && press.isPressed(lastGamepad)){
                    onPress.run();
                }
                break;
        }


        return Collections.emptyList();
    }

    public enum Behavior {
        RISING_EDGE, WHILE_HELD, FALLING_EDGE
    }
}
