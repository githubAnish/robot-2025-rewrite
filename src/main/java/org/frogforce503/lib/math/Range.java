package org.frogforce503.lib.math;

import edu.wpi.first.math.MathUtil;

/** A range of 2 numbers, with a min and max value. */
public record Range(
    double min,
    double max
) {
    public Range() {
        this(0.0, 0.0);
    }

    public double clamp(double value) {
        return MathUtil.clamp(value, min, max);
    }

    public boolean contains(double value) {
        return min <= value && value <= max;
    }
}