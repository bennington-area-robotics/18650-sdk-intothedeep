package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class DriveBaseMotor implements DcMotor {
    public double getPositionDegrees(){
        return (getCurrentPosition() / 28.0) * 360.0;
    }
}
