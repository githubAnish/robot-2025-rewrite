package org.frogforce503.lib.math;

import java.util.Arrays;

import edu.wpi.first.math.MathUtil;

/**
 * Miscellaneous math utility functions.
 * 
 * @see {@link MathUtil} for more common math utility functions.
 */
public final class MathUtils {
    private MathUtils() {}

    /** Finds the minimum of {@code values}. */
    public static double minOf(double... values) {
        return
            Arrays
                .stream(values)
                .min()
                .orElse(Double.MAX_VALUE);
    }

    /** Finds max of {@code values} */
    public static double maxOf(double... values) {
        return
            Arrays
                .stream(values)
                .max()
                .orElse(Double.MIN_VALUE);
    }

    /** Rounds a number to a specified decimal places. */
    public static double roundTo(double num, double digits) {
        return Math.round(num * Math.pow(10, digits)) / Math.pow(10, digits);
    }

    public static boolean equalsOneOf(double toCheckValue, double... options) {
        return
            Arrays
                .stream(options)
                .anyMatch(v -> v == toCheckValue);
    }

    /**
     * Returns if the value is in the range [lowerBound, upperBound].
     *
     * @param lowerBound The lower bound of the range.
     * @param upperBound The upper bound of the range.
     * @param value      The value.
     * @return If the value is in the range.
     */
    public static boolean isBetween(double lowerBound, double upperBound, double value) {
        return lowerBound <= value && value <= upperBound;
    }

    public static boolean isInRange(double measurement, Range range) {
        return isBetween(range.min(), range.max(), measurement);
    }

    /**
     * Solves the equation <code>0 = ax<sup>2</sup> + bx + c</code> for x and
     * returns the real results.
     *
     * @param a the a coefficient
     * @param b the b coefficient
     * @param c the c coefficient
     * @return the real roots of the equation
     */
    public static double[] quadratic(double a, double b, double c) {
        double discriminant = Math.sqrt(b * b - 4 * a * c);
        if (Double.isNaN(discriminant)) {
            return new double[0]; // No roots
        }

        return
            new double[] {
                (-b + discriminant) / (2 * a),
                (-b - discriminant) / (2 * a)};
    }
}
