package org.frogforce503.lib.math;

import java.util.Arrays;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

/**
 * Miscellaneous math functions, use {@link MathUtil} for more common functions
 */
public final class MathUtils {
    private MathUtils() {}

    /** Finds min of {@code values} */
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
     * @since 0.1
     */
    public static boolean isBetween(double lowerBound, double upperBound, double value) {
        return lowerBound <= value && value <= upperBound;
    }

    public static boolean isInRange(double measurement, Range range) {
        return isBetween(range.min(), range.max(), measurement);
    }

    /**
     * More efficient hypot method without the square root, generally for the purpose of checking if points are within a distance
     * @param x
     * @param y
     * @return c^2
     */
    public static double hypotSquared(double x, double y) {
        return x * x + y * y;
    }

    public static double distanceSquared(Translation2d a, Translation2d b) {
        return hypotSquared(a.getX() - b.getX(), a.getY() - b.getY());
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

    /**
     * Returns the cross product of two vectors (of type Translation2d).
     * 
     * @param v
     * @param w
     * @return Cross product of {@code v} and {@code w}
     */
    public static double cross(Translation2d v, Translation2d w) {
        return v.getX() * w.getY() - v.getY() * w.getX();
    }

    /**
     * 
     * See:
     * https://github.com/Team254/FRC-2022-Public/blob/6a24236b37f0fcb75ceb9d5dec767be58ea903c0/src/main/java/com/team254/lib/geometry/Pose2d.java#L82
     * and https://github.com/strasdat/Sophus/blob/master/sophus/se2.hpp
     */
    public static Twist2d poseLog(final Pose2d transform) {
        final double dtheta = transform.getRotation().getRadians();
        final double half_dtheta = 0.5 * dtheta;
        final double cos_minus_one = transform.getRotation().getCos() - 1.0;
        double halftheta_by_tan_of_halfdtheta;
        if (Math.abs(cos_minus_one) < 1E-9) {
            halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        } else {
            halftheta_by_tan_of_halfdtheta = -(half_dtheta * transform.getRotation().getSin()) / cos_minus_one;
        }
        final Translation2d translation_part = transform.getTranslation()
                .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta));
        return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
    }
}
