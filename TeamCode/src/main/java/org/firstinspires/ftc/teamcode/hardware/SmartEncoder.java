package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.roadrunner.util.Encoder;

public class SmartEncoder extends Encoder implements Caching{
    private final HardwareCache<Integer> positionCache;
    private final HardwareCache<Double> velocityCache;
    private final HardwareCache<Double> correctedVelocityCache;

    SmartEncoder(DcMotorEx motor) {
        super(motor);
        this.positionCache = new HardwareCache<>(super::getCurrentPosition);
        this.velocityCache = new HardwareCache<>(super::getRawVelocity);
        this.correctedVelocityCache = new HardwareCache<>(super::getCorrectedVelocity);
    }

    @Override
    public int getCurrentPosition() {
        return positionCache.read();
    }


    /**
     *
     */
    @Override
    public void invalidateCache() {
        positionCache.invalidateCache();
        velocityCache.invalidateCache();
        correctedVelocityCache.invalidateCache();
    }

    /**
     *
     */
    @Override
    public void updateCache() {
        positionCache.updateCache();
        velocityCache.updateCache();
        correctedVelocityCache.updateCache();
    }

    @Override
    public void setStrategy(CachingStrategy strategy) {
        positionCache.setStrategy(strategy);
        velocityCache.setStrategy(strategy);
        correctedVelocityCache.setStrategy(strategy);
    }

    @Override
    public CachingStrategy getStrategy() {
        return positionCache.getStrategy();
    }
}
