package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.robotcore.external.Supplier;

public class HardwareCache<T> implements Caching {
    private T cachedValue;
    private boolean cacheValid = false;
    private CachingStrategy strategy = CachingStrategy.UPDATE_WHEN_INVALIDATED;
    private final Supplier<T> valueSupplier;
    private boolean cacheRead = false;

    public HardwareCache(Supplier<T> valueSupplier){
        this.valueSupplier = valueSupplier;
    }

    @Override
    public void invalidateCache() {
        if(strategy == CachingStrategy.UPDATE_WHEN_INVALIDATED) {
            updateCache();
        } else
            this.cacheValid = false;

        cacheRead = false;
    }

    @Override
    public void updateCache() {
        this.cachedValue = valueSupplier.get();
        cacheValid = true;
        cacheRead = false;
    }

    public T read() {
        if(cachedValue == null || !cacheValid){
            updateCache();
        }

        switch (strategy){
            case UPDATE_WHEN_INVALIDATED:
            case VALID_UNTIL_INVALIDATED:
                cacheRead = true;
                return cachedValue;
            case INVALID_AFTER_FIRST_READ:
                if(cacheRead){
                    updateCache();
                }else {
                    cacheRead = true;
                }
                return cachedValue;
        }

        return cachedValue;
    }

    public boolean isCacheValid() {
        return cacheValid;
    }

    public void setStrategy(CachingStrategy strategy){
        this.strategy = strategy;
        cacheValid = false;
        cacheRead = false;
    }

    public CachingStrategy getStrategy() {
        return strategy;
    }
}
