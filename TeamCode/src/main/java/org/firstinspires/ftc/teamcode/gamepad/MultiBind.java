package org.firstinspires.ftc.teamcode.gamepad;

import java.util.List;

public abstract class MultiBind implements Bind {
    protected final GamePad.Press control;
    protected boolean bubble = false;

    MultiBind(GamePad.Press control){
        assert control != null;
        this.control = control;
    }

    /**
     * Allows presses and values caught by this multibind to continue to trigger other multibinds and regular binds.
     *
     * @return this multibind.
     */
    public MultiBind allowBubble(){
        this.bubble = true;
        return this;
    }

    public abstract List<Bindable> getBlockedBindables();
}
