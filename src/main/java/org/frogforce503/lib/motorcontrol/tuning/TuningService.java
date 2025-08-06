package org.frogforce503.lib.motorcontrol.tuning;

public interface TuningService<T> {
    void setTuning(boolean enabled);
    boolean isTuningEnabled();
    T getUpdatedConfig();
}