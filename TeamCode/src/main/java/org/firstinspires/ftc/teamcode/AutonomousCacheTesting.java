package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
@Config
public class AutonomousCacheTesting extends AutoTemplate {


//TODO FOR EBEN - clean up this code! remove the unnecessary code if its commented, implement the methods I added here

    public static double blueStartX = 0;
    public static double blueStartY = 63;
    public static double blueStartAng = 90;

    public static double redStartX = 0;
    public static double redStartY = -63;
    public static double redStartAng = -90;

    private static ElapsedTime runtime = new ElapsedTime();

    public static double specX = 0, specY = 40, specTan = 90, specHeading = 90;
    public static double collectorPos = 40, armAnglePos = 45, armExtensionPos = 16.3;

    @Override
    public void runOpMode() throws InterruptedException {
        setBlueStartPose(blueStartX, blueStartY, blueStartAng);
        super.initialize();

        waitForStart();

        run();
    }

    @Override
    public void initialize(){

    }
    @Override
    public void run(){


        if (isStopRequested()) return;
        runtime.reset();
        collector.closeGrip();
        collector.wristToDefaultPosition();
        performWithManualCaching(() -> arm.moveToTargetAngleBlocking(armAnglePos, this::tickArm));
        while(runtime.seconds() < 30 && opModeIsActive()){
            tickAll();
        }
    }
}
