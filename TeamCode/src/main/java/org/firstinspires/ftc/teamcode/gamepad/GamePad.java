package org.firstinspires.ftc.teamcode.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.List;

public class GamePad {
    private final Gamepad baseGamepad;

    private Gamepad lastGp;

    private final List<MonoBind> monoBinds;
    private final List<MultiBind> multiBinds;

    GamePad(Gamepad baseGamepad, List<MonoBind> monoBinds, List<MultiBind> multiBinds){
        this.baseGamepad = baseGamepad;
        this.monoBinds = monoBinds;
        this.multiBinds = multiBinds;
    }

    public void check(){
        Gamepad gp = new Gamepad();
        gp.copy(baseGamepad);

        List<Bindable> blockedBindables = new ArrayList<>();

        for(MultiBind bind : multiBinds){
            blockedBindables.addAll(bind.accept(gp, lastGp, blockedBindables));
        }

        for (MonoBind bind : monoBinds) {
            bind.accept(gp, lastGp, blockedBindables);
        }

        lastGp.copy(gp);
    }

    public boolean isPressed(Press press){
        return press.isPressed(baseGamepad);
    }

    public float getValue(Value value){
        return value.get(baseGamepad);
    }



    public enum Press implements Bindable {
        A, B, X, Y, BACK, START, GUIDE,
        L_BUMPER, R_BUMPER, L_STICK_BTN, R_STICK_BTN, DPAD_LEFT, DPAD_UP, DPAD_RIGHT, DPAD_DOWN,
        CIRCLE, CROSS, TRIANGLE, SQUARE, SHARE, OPTIONS, PS, TOUCHPAD, TOUCH_1, TOUCH_2; //playstation ONLY
        
        public boolean isPressed(Gamepad gp){
            switch (this){
                case A: return gp.a;
                case B: return gp.b;
                case X: return gp.x; 
                case Y: return gp.y;
                case BACK: return gp.back; 
                case START: return gp.start; 
                case GUIDE: return gp.guide; 
                case L_BUMPER: return gp.left_bumper; 
                case R_BUMPER: return gp.right_bumper; 
                case L_STICK_BTN: return gp.left_stick_button; 
                case R_STICK_BTN: return gp.right_stick_button; 
                case DPAD_LEFT: return gp.dpad_left; 
                case DPAD_UP: return gp.dpad_up; 
                case DPAD_RIGHT: return gp.dpad_right; 
                case DPAD_DOWN: return gp.dpad_down; 
                case CIRCLE: return gp.circle; 
                case CROSS: return gp.cross; 
                case TRIANGLE: return gp.triangle; 
                case SQUARE: return gp.square; 
                case SHARE: return gp.share; 
                case OPTIONS: return gp.options; 
                case PS: return gp.ps; 
                case TOUCHPAD: return gp.touchpad; 
                case TOUCH_1: return gp.touchpad_finger_1; 
                case TOUCH_2: return gp.touchpad_finger_2; 
            }

            return false;
        }
    }

    public enum Value implements Bindable {
        L_STICK_X, L_STICK_Y, R_STICK_X, R_STICK_Y, L_TRIGGER, R_TRIGGER,
        TOUCH_1_X, TOUCH_1_Y, TOUCH_2_X, TOUCH_2_Y; //playstation ONLY

        public float get(Gamepad gp){
            switch (this){
                case L_STICK_X: return gp.left_stick_x;
                case L_STICK_Y: return gp.left_stick_y;
                case R_STICK_X: return gp.right_stick_x;
                case R_STICK_Y: return gp.right_stick_y;
                case L_TRIGGER: return gp.left_trigger;
                case R_TRIGGER: return gp.right_trigger;
                case TOUCH_1_X: return gp.touchpad_finger_1_x;
                case TOUCH_1_Y: return gp.touchpad_finger_1_y;
                case TOUCH_2_X: return gp.touchpad_finger_2_x;
                case TOUCH_2_Y: return gp.touchpad_finger_2_y;
            }

            return 0;
        }
    }
}
