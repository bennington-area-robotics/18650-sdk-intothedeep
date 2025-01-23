package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Collector;
import org.firstinspires.ftc.teamcode.hardware.DriveBase;

public class Autopilot {
    private final DriveBase driveBase;
    private final Arm arm;
    private final Collector collector;
    private boolean isRunning;
    private Runnable tickRunnable;

    public Autopilot(DriveBase driveBase, Arm arm, Collector collector){
        this.driveBase = driveBase;
        this.arm = arm;
        this.collector = collector;
    }

    public void setTickRunnable(Runnable runnable){
        tickRunnable = runnable;
    }

    public void stop(){
        isRunning = false;
    }

    public void start(){
        isRunning = true;
    }

    public enum Stage{
        AWAIT_USER_INTERVENTION,
        MOVE_TO_BASKET, PREPARE_DELIVERY,
        AWAIT_USER_DELIVERY_CONFIRMATION, DELIVER_SAMPLE,
        LOWER_ARM, RETURN_TO_SUBMERSIBLE,
    }
}
