package org.firstinspires.ftc.teamcode.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class DoublePressBind extends MultiBind{
    private final GamePad.Press press;
    private final Runnable onPress;
    private final Behavior behavior;

    public DoublePressBind(GamePad.Press control, GamePad.Press press, Behavior behavior, Runnable onPress){
        super(control);
        assert press != null; assert behavior != null; assert onPress != null;
        this.press = press;
        this.behavior = behavior;
        this.onPress = onPress;
    }

    public enum Behavior {
        WHILE_BOTH_HELD, CONTROL_HELD_AND_PRESS_RISING_EDGE, CONTROL_HELD_AND_PRESS_FALLING_EDGE,
    }

    public List<Bindable> accept(Gamepad currentGamepad, Gamepad lastGamepad, List<Bindable> blockedBindables){
        if(blockedBindables.contains(control) || blockedBindables.contains(press))
            return Collections.emptyList();

        boolean controlPressed = control.isPressed(currentGamepad);
        switch (behavior){
            case WHILE_BOTH_HELD:
                if(controlPressed && press.isPressed(currentGamepad)){
                    onPress.run();
                }
                break;
            case CONTROL_HELD_AND_PRESS_RISING_EDGE:
                if(controlPressed && (press.isPressed(currentGamepad) && !press.isPressed(lastGamepad))){
                    onPress.run();
                }
                break;
            case CONTROL_HELD_AND_PRESS_FALLING_EDGE:
                if(controlPressed && (!press.isPressed(currentGamepad) && press.isPressed(lastGamepad))){
                    onPress.run();
                }
                break;
        }
        if(controlPressed){
            return Collections.singletonList(press);
        }

        return Collections.emptyList();
    }

    @Override
    public List<Bindable> getBlockedBindables() {
        if(bubble)
            return Collections.emptyList();
        else
            return Arrays.asList(control, press);
    }
}