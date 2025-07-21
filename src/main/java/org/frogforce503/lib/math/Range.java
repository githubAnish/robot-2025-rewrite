package org.frogforce503.lib.math;

public record Range(
    double min,
    double max
) {
    public Range() {
        this(0.0, 0.0);
    }
}