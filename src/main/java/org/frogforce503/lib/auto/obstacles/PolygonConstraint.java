package org.frogforce503.lib.auto.obstacles;

import org.frogforce503.lib.math.Polygon2d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;

/** Enforces a particular constraint only within a polygonal region. */
public class PolygonConstraint implements TrajectoryConstraint {
  public final Polygon2d m_polygon;
  public final TrajectoryConstraint m_constraint;

  /**
   * Constructs a new TrajConstraint.
   *
   * @param rectangle The rectangular region in which to enforce the constraint.
   * @param constraint The constraint to enforce when the robot is within the region.
   */
  public PolygonConstraint(Polygon2d polygon, TrajectoryConstraint constraint) {
    m_polygon = polygon;
    m_constraint = constraint;
  }

  public double getMaxVelocityMetersPerSecond(
      Pose2d poseMeters, double curvatureRadPerMeter, double velocityMetersPerSecond, double overrideVel) {
    if (m_polygon.contains(poseMeters.getTranslation())) {
      return m_constraint.getMaxVelocityMetersPerSecond(
          poseMeters, curvatureRadPerMeter, velocityMetersPerSecond);
    } else {
      return overrideVel;
    }
  }

  @Override
  public double getMaxVelocityMetersPerSecond(
      Pose2d poseMeters, double curvatureRadPerMeter, double velocityMetersPerSecond) {
    return getMaxVelocityMetersPerSecond(poseMeters, curvatureRadPerMeter, velocityMetersPerSecond, Double.POSITIVE_INFINITY);
  }

  public double getMaxVelocityMetersPerSecond(
      Pose2d poseMeters, double overrideVel) {
    return getMaxVelocityMetersPerSecond(poseMeters, 0, 0, overrideVel);
  }

  @Override
  public MinMax getMinMaxAccelerationMetersPerSecondSq(
      Pose2d poseMeters, double curvatureRadPerMeter, double velocityMetersPerSecond) {
    if (m_polygon.contains(poseMeters.getTranslation())) {
      return m_constraint.getMinMaxAccelerationMetersPerSecondSq(
          poseMeters, curvatureRadPerMeter, velocityMetersPerSecond);
    } else {
      return new MinMax();
    }
  }
}
