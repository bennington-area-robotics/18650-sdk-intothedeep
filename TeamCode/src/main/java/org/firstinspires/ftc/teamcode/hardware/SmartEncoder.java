package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.roadrunner.util.Encoder;

public class SmartEncoder implements Caching{
    private final HardwareCache<Integer> positionCache;
    private final HardwareCache<Double> velocityCache;

    private Encoder rrEncoder;

    Direction direction = Direction.FORWARD;

    //todo add reset()

    SmartEncoder(DcMotorEx motor) {
        this.positionCache = new HardwareCache<>(motor::getCurrentPosition);
        this.velocityCache = new HardwareCache<>(motor::getVelocity);
    }

    SmartEncoder(Encoder rrEncoder){
        this.rrEncoder = rrEncoder;
        this.direction = Direction.of(rrEncoder.getDirection());
        this.positionCache = new HardwareCache<>(this.rrEncoder::getCurrentPosition);
        this.velocityCache = new HardwareCache<>(this.rrEncoder::getCorrectedVelocity);
    }

    public int getPosition() {
        return positionCache.read();
    }

    public double getVelocity() {
        return velocityCache.read();
    }

    @Override
    public void invalidateCache() {
        positionCache.invalidateCache();
        velocityCache.invalidateCache();
    }

    @Override
    public void updateCache() {
        positionCache.updateCache();
        velocityCache.updateCache();
    }

    @Override
    public void setStrategy(CachingStrategy strategy) {
        positionCache.setStrategy(strategy);
        velocityCache.setStrategy(strategy);
    }

    @Override
    public CachingStrategy getStrategy() {
        return positionCache.getStrategy();
    }

    public void setDirection(Direction direction){
        if(rrEncoder != null)
            rrEncoder.setDirection(direction.toRR());
        this.direction = direction;
    }

    public Direction getDirection() {
        return direction;
    }
}
