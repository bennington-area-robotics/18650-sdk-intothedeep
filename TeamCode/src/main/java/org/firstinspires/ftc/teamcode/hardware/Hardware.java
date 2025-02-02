package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

public class Hardware {
    private static final List<SmartMotor> motors = new ArrayList<>();
    private static HardwareMap hardwareMap;
    private static List<LynxModule> hubs;

    public void init(HardwareMap hardwareMap){
        Hardware.hardwareMap = hardwareMap;
        Hardware.hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
    }

    public SmartMotor getMotor(HardwareMap hardwareMap, String name){
        SmartMotor smartMotor = new SmartMotor(hardwareMap.get(DcMotor.class, name));
        motors.add(smartMotor);
        return smartMotor;
    }

    public void updateAllCaches(){
        hubs.forEach(LynxModule::clearBulkCache);
    }
}
