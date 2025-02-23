package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.robotcore.external.Supplier;

public class HardwareCache<T> implements Caching {
    private T cachedValue;
    private boolean cacheValid;
    private CachingStrategy strategy;
    private final Supplier<T> valueSupplier;
    private boolean cacheRead = false;

    public HardwareCache(Supplier<T> valueSupplier){
        this.valueSupplier = valueSupplier;
    }

    @Override
    public void invalidateCache() {
        if(strategy == CachingStrategy.UPDATE_WHEN_INVALIDATED)
            updateCache();
        else
            this.cacheValid = false;
    }

    @Override
    public void updateCache() {
        this.cachedValue = valueSupplier.get();
        cacheRead = false;
    }

    public T read() {
        switch (strategy){
            case UPDATE_WHEN_INVALIDATED:
            case VALID_UNTIL_INVALIDATED:
                cacheRead = true;
                return readFromOrUpdateCache();
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

    private T readFromOrUpdateCache(){
        if(!cacheValid){
            updateCache();
        }
        return cachedValue;
    }

    public boolean isCacheValid() {
        return cacheValid;
    }

    public void setStrategy(CachingStrategy strategy){
        this.strategy = strategy;
    }

    public CachingStrategy getStrategy() {
        return strategy;
    }
}
