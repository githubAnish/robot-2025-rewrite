package org.frogforce503.lib.auto.follower;

import org.frogforce503.lib.planning.planned_path.PlannedPath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import lombok.Getter;
import lombok.Setter;

public class SwervePathFollower {
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;

    @Getter private Pose2d poseError = Pose2d.kZero;
    @Getter private Rotation2d rotationError = Rotation2d.kZero;
    @Setter private Pose2d poseTolerance = Pose2d.kZero;

    public SwervePathFollower(PIDController xController, PIDController yController, PIDController thetaController){
        this.xController = xController;
        this.yController = yController;
        this.thetaController = thetaController;

        // Enable continuous input for theta controller
        this.thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public SwervePathFollower(AutoPIDController autoPIDController) {
        this(
            autoPIDController.autoXController(),
            autoPIDController.autoYController(),
            autoPIDController.autoThetaController());
    }

    public void reset() {
        this.xController.reset();
        this.yController.reset();
        this.thetaController.reset();
    }

    public boolean atReference() {
        final var eTranslate = poseError.getTranslation();
        final var eRotate = rotationError;
        final var tolTranslate = poseTolerance.getTranslation();
        final var tolRotate = poseTolerance.getRotation();

        return
            Math.abs(eTranslate.getX()) < tolTranslate.getX() &&
            Math.abs(eTranslate.getY()) < tolTranslate.getY() &&
            Math.abs(eRotate.getRadians()) < tolRotate.getRadians();
    }

    public ChassisSpeeds calculate(
        final Pose2d currentPose,
        final Pose2d targetPose,
        final Rotation2d holonomicAngle,
        final double xVelocityRef,
        final double yVelocityRef,
        final double thetaVelocityRef
    ) {
        poseError = targetPose.relativeTo(currentPose);
        rotationError = holonomicAngle.minus(currentPose.getRotation());

        // Calculate feedback velocities (based on position error).
        final double xFeedback = xController.calculate(currentPose.getX(), targetPose.getX());
        final double yFeedback = yController.calculate(currentPose.getY(), targetPose.getY());
        final double thetaFeedback =
            thetaController.calculate(
                currentPose.getRotation().getRadians(), holonomicAngle.getRadians());

        // Return next output.
        return
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xVelocityRef + xFeedback,
                yVelocityRef + yFeedback,
                thetaVelocityRef + thetaFeedback,
                currentPose.getRotation());
    }

    public ChassisSpeeds calculate(
        final Pose2d currentPose,
        final Pose2d targetPose,
        final double xVelocityRef,
        final double yVelocityRef,
        final double thetaVelocityRef
    ) {
        poseError = targetPose.relativeTo(currentPose);
        rotationError = poseError.getRotation();

        // Calculate feedback velocities (based on position error).
        final double xFeedback = xController.calculate(currentPose.getX(), targetPose.getX());
        final double yFeedback = yController.calculate(currentPose.getY(), targetPose.getY());
        final double thetaFeedback =
            thetaController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

        // Return next output.
        return
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xVelocityRef + xFeedback,
                yVelocityRef + yFeedback,
                thetaVelocityRef + thetaFeedback,
                currentPose.getRotation());
    }

    public ChassisSpeeds calculate(
        Pose2d currentPose,
        Pose2d poseRef,
        Rotation2d angleRef,
        double linearVelocityRefMeters,
        double angleVelocityRefRadians
    ) {
        return
            calculate(
                currentPose,
                poseRef,
                angleRef,
                linearVelocityRefMeters * poseRef.getRotation().getCos(),
                linearVelocityRefMeters * poseRef.getRotation().getSin(),
                angleVelocityRefRadians);
    }

    public ChassisSpeeds calculate(Pose2d currentPose, PlannedPath.HolonomicState state) {
        return
            calculate(
                currentPose,
                state.poseMeters(),
                state.holonomicAngle(),
                state.velocityMetersPerSecond(),
                state.angularVelocityRadiansPerSec());
    }
}
