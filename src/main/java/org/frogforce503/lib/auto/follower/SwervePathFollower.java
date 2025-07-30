package org.frogforce503.lib.auto.follower;

import org.frogforce503.lib.auto.trajectory.path.PlannedPath;

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

    private Translation2d translationError = Translation2d.kZero;
    private Rotation2d rotationError = Rotation2d.kZero;
    @Setter private Pose2d tolerance = Pose2d.kZero;

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

    public SwervePathFollower(AutoPIDController autoPIDController) {
        this(
            autoPIDController.autoXController(),
            autoPIDController.autoYController(),
            autoPIDController.autoThetaController());
    }

    /** Resets the controllers and clears the pose error. */
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

    /**
     * Calculates the next chassis speeds for the robot to follow the desired trajectory.
     *
     * @param currentPose The current pose of the robot on the field.
     * @param poseRef The desired reference pose for the robot to follow.
     * @param velXMeters The desired velocity in the field-relative X direction (meters per second).
     * @param velYMeters The desired velocity in the field-relative Y direction (meters per second).
     * @param angleRef The desired reference rotation for the robot.
     * @param angleVelocityRefRadians The desired angular velocity (radians per second).
     * @return The calculated {@link ChassisSpeeds} to achieve the desired motion.
     */
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
        double xFeedback =
            xController.calculate(currentPose.getX(), poseRef.getX());
        double yFeedback =
            yController.calculate(currentPose.getY(), poseRef.getY());
        double thetaFeedback =
            rotationController.calculate(currentPose.getRotation().getRadians(), angleRef.getRadians());

        // Return next output.
        return
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xFF + xFeedback,
                yFF + yFeedback,
                thetaFF + thetaFeedback,
                currentPose.getRotation());
    }

    /**
     * Calculates the next chassis speeds for the robot to follow the desired trajectory.
     *
     * @param currentPose The current pose of the robot on the field.
     * @param poseRef The reference pose the robot should follow.
     * @param linearVelocityRefMeters The desired linear velocity in meters per second.
     * @param angleRef The desired orientation of the robot as a {@link Rotation2d}.
     * @param angleVelocityRefRadians The desired angular velocity in radians per second.
     * @return The calculated {@link ChassisSpeeds} to achieve the desired motion.
     */
    public ChassisSpeeds calculate(
        Pose2d currentPose,
        Pose2d poseRef,
        double linearVelocityRefMeters,
        Rotation2d angleRef,
        double angleVelocityRefRadians
    ) {
        return
            calculate(
                currentPose,
                poseRef,
                linearVelocityRefMeters * poseRef.getRotation().getCos(),
                linearVelocityRefMeters * poseRef.getRotation().getSin(),
                angleRef,
                angleVelocityRefRadians);
    }

    /**
     * Calculates the next chassis speeds for the robot to follow the desired trajectory.
     *
     * @param currentPose The current pose.
     * @param driveState The desired drive trajectory state.
     * @param holonomicRotationState The desired holonomic rotation state.
     * @return The calculated {@link ChassisSpeeds} to achieve the desired motion.
     */
    public ChassisSpeeds calculate(Pose2d currentPose, PlannedPath.HolonomicState state) {
        return
            calculate(
                currentPose,
                state.poseMeters(),
                state.velocityMetersPerSecond(),
                state.holonomicAngle(),
                state.angularVelocityRadiansPerSec());
    }
}
