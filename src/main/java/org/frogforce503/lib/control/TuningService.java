package org.frogforce503.lib.control;

public interface TuningService<T> {
    void setTuning(boolean enabled);
    boolean isTuningEnabled();
    T getUpdatedConfig();
}