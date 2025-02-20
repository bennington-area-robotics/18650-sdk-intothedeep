package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.teamcode.utilities.Pose;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class Hardware {
    //todo give each hardware device a wrapper with caching strategies.
    private static final List<SmartCamera> cameras = new ArrayList<>();
    private static final List<SmartMotor> motors = new ArrayList<>();
    private static final List<SmartColorSensor> colorSensors = new ArrayList<>();
    private static final List<SmartTouchSensor> touchSensors = new ArrayList<>();
    private static final List<SmartServo> servos = new ArrayList<>();

    private static HardwareMap hardwareMap;
    private static List<LynxModule> hubs;

    public static void init(HardwareMap hardwareMap) {
        Hardware.hardwareMap = hardwareMap;
        Hardware.hubs = hardwareMap.getAll(LynxModule.class);
    }

    public static List<LynxModule> getHubs() {
        assertInitialized();
        return hubs;
    }

    public static SmartCamera getCamera(String name, Pose pose){
        assertInitialized();
        Optional<SmartCamera> hardwareOptional = cameras.stream().filter(camera -> camera.getName().equals(name)).findAny();
        if (hardwareOptional.isPresent())
            return hardwareOptional.get();

        SmartCamera camera = new SmartCamera(hardwareMap.get(CameraName.class, name), name, pose);
        cameras.add(camera);
        return camera;
    }

    /**
     * Retrieves the hardware object for the given motor. If the motor has already been retrieved, this will return the same cached motor object.
     *
     * @param name the name of the motor.
     * @return the motor object associated with the passed name.
     */
    public static SmartMotor getMotor(String name) {
        assertInitialized();
        Optional<?> hardwareOptional = getHardwareOptional(motors, name);
        if (hardwareOptional.isPresent())
            return (SmartMotor) hardwareOptional.get();

        SmartMotor smartMotor = new SmartMotor(hardwareMap.get(DcMotor.class, name));
        motors.add(smartMotor);
        return smartMotor;
    }

    public static SmartColorSensor getColorSensor(String name) {
        assertInitialized();
        Optional<?> hardwareOptional = getHardwareOptional(colorSensors, name);
        if(hardwareOptional.isPresent())
            return (SmartColorSensor) hardwareOptional.get();
        
        SmartColorSensor smartColorSensor =  new SmartColorSensor(hardwareMap.get(NormalizedColorSensor.class, name));
        colorSensors.add(smartColorSensor);
        return smartColorSensor;
    }
    
    public static SmartServo getServo(String name){
        assertInitialized();

        Optional<?> hardwareOptional = getHardwareOptional(servos, name);
        if(hardwareOptional.isPresent())
            return (SmartServo) hardwareOptional.get();

        SmartServo servo = new SmartServo(hardwareMap.get(Servo.class, name));
        servos.add(servo);
        return servo;
    }

    public static SmartTouchSensor getTouchSensor(String name){
        Optional<?> hardwareOptional = getHardwareOptional(touchSensors, name);
        if(hardwareOptional.isPresent())
            return (SmartTouchSensor) hardwareOptional.get();

        SmartTouchSensor smartTouchSensor = new SmartTouchSensor(hardwareMap.get(TouchSensor.class, name));
        touchSensors.add(smartTouchSensor);
        return smartTouchSensor;
    }

    /**
     * @param type the hardware class which you would like to get. Only supports FTC-SDK hardware.
     * @param name the name of the device to get.
     * @return the hardware object requested.
     */
    public static <T> T getOther(Class<? extends T> type, String name) {
        return hardwareMap.get(type, name);
    }

    public void invalidateCaches() {
        motors.forEach(SmartMotor::invalidateCache);
    }

    private static void assertInitialized() {
        if (hardwareMap == null)
            throw new HardwareMapNotInitializedException();
    }

    private static Optional<? extends HardwareDevice> getHardwareOptional(List<? extends HardwareDevice> list, String name){
        return list.stream().filter(colorSensor -> colorSensor.getDeviceName().equals(name)).findAny();
    }
}
