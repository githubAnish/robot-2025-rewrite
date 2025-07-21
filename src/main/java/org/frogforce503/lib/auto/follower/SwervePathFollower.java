package org.frogforce503.lib.auto.follower;

import org.frogforce503.lib.trajectory.PlannedPath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import lombok.Getter;
import lombok.Setter;

public class SwervePathFollower {
    @Getter private final PIDController xController;
    @Getter private final PIDController yController;
    @Getter private final PIDController rotationController;

    private Translation2d translationError = new Translation2d();
    private Rotation2d rotationError = Rotation2d.kZero;
    @Setter private Pose2d tolerance = new Pose2d();

    /**
     * 
     * @param xController A PID controller to respond to error in the field-relative X direction
     * @param yController A PID controller to respond to error in the field-relative Y direction
     * @param rotationController A PID controller to respond to error in rotation
     */
    public SwervePathFollower(PIDController xController, PIDController yController, PIDController rotationController){
        this.xController = xController;
        this.yController = yController;
        this.rotationController = rotationController;

        // Enable continuous input for theta controller
        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void reset() {
        this.xController.reset();
        this.yController.reset();
        this.rotationController.reset();
    }

    /** Returns true if the pose error is within tolerance of the reference. */
    public boolean atReference() {
        Translation2d translationTolerance = this.tolerance.getTranslation();
        Rotation2d rotationTolerance = this.tolerance.getRotation();

        return
            Math.abs(this.translationError.getX()) < translationTolerance.getX() &&
            Math.abs(this.translationError.getY()) < translationTolerance.getY() &&
            Math.abs(this.rotationError.getRadians()) < rotationTolerance.getRadians();
    }

    public ChassisSpeeds calculate(
        Pose2d currentPose,
        Pose2d poseRef,
        double velXMeters,
        double velYMeters,
        Rotation2d angleRef,
        double angleVelocityRefRadians
    ) {
        // Calculate feedforward velocities (field-relative).
        double xFF = velXMeters;
        double yFF = velYMeters;
        double thetaFF = angleVelocityRefRadians;

        translationError = poseRef.relativeTo(currentPose).getTranslation();
        rotationError = angleRef.minus(currentPose.getRotation());

        // Calculate feedback velocities (based on position error).
        double xFeedback = xController.calculate(currentPose.getX(), poseRef.getX());
        double yFeedback = yController.calculate(currentPose.getY(), poseRef.getY());
        double thetaFeedback =
            rotationController.calculate(currentPose.getRotation().getRadians(), angleRef.getRadians());

        // Return next output.
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            xFF + xFeedback, yFF + yFeedback, thetaFF + thetaFeedback, currentPose.getRotation());
    }

    public ChassisSpeeds calculate(
        Pose2d currentPose,
        Pose2d poseRef,
        double linearVelocityRefMeters,
        Rotation2d angleRef,
        double angleVelocityRefRadians
    ) {
        return calculate(
            currentPose,
            poseRef,
            linearVelocityRefMeters * poseRef.getRotation().getCos(),
            linearVelocityRefMeters * poseRef.getRotation().getSin(),
            angleRef,
            angleVelocityRefRadians);
    }

    /**
     * Returns the next output of the holonomic drive controller.
     *
     * @param currentPose The current pose.
     * @param driveState The desired drive trajectory state.
     * @param holonomicRotationState The desired holonomic rotation state.
     * @return The next output of the holonomic drive controller.
     */
    public ChassisSpeeds calculate(Pose2d currentPose, PlannedPath.HolonomicState state) {
        return calculate(
            currentPose,
            state.poseMeters(),
            state.velocityMetersPerSecond(),
            state.holonomicAngle(),
            state.angularVelocityRadiansPerSec());
    }
}
