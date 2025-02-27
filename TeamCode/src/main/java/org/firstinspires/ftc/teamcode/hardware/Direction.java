package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.teamcode.drive.roadrunner.util.Encoder;

public enum Direction {
    FORWARD, REVERSE;

    public static Direction of(Encoder.Direction theirs){
        return theirs == Encoder.Direction.FORWARD ? FORWARD : REVERSE;
    }

    public Encoder.Direction toRR(){
        return this == FORWARD ? Encoder.Direction.FORWARD : Encoder.Direction.REVERSE;
    }
}
