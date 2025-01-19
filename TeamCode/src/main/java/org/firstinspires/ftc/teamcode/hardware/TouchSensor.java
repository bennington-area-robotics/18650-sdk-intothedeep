package org.firstinspires.ftc.teamcode.hardware;
import android.text.method.Touch;

import com.qualcomm.robotcore.hardware.HardwareMap;
public class TouchSensor {
    private TouchSensor touchSensor;

    public TouchSensor(HardwareMap hardwareMap, String name){
        this.touchSensor = hardwareMap.get(TouchSensor.class, name);

    }
    public double isPressed (){
        return touchSensor.isPressed();
    }
}
