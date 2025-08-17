package org.frogforce503.lib.auto.follower;

import org.frogforce503.lib.planning.planned_path.PlannedPath.HolonomicState;

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

    public SwervePathFollower(PIDController xController, PIDController yController, PIDController thetaController) {
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

    /** Calculates the next {@link ChassisSpeeds} for the robot to follow based on the following parameters:
     * @param currentPose Current pose of the robot
     * @param targetPose Desired pose the robot needs to go to
     * @param targetAngle Desired angle the robot needs to turn to
     * @param xFF Desired x velocity in m/s
     * @param yFF Desired y velocity in m/s
     * @param thetaFF Desired theta velocity in rad/s
     */
    public ChassisSpeeds calculate(
        final Pose2d currentPose,
        final Pose2d targetPose,
        final Rotation2d targetAngle,
        final double xFF,
        final double yFF,
        final double thetaFF
    ) {
        // Update errors
        poseError = targetPose.relativeTo(currentPose);
        rotationError = targetAngle.minus(currentPose.getRotation());

        // Calculate feedback velocities (based on position error).
        final double xFeedback = xController.calculate(currentPose.getX(), targetPose.getX());
        final double yFeedback = yController.calculate(currentPose.getY(), targetPose.getY());
        final double thetaFeedback =
            thetaController.calculate(
                currentPose.getRotation().getRadians(), targetAngle.getRadians());

        // Return next output.
        return
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xFF + xFeedback,
                yFF + yFeedback,
                thetaFF + thetaFeedback,
                currentPose.getRotation());
    }

    public ChassisSpeeds calculate(
        final Pose2d currentPose,
        final Pose2d targetPose,
        final double xFF,
        final double yFF,
        final double thetaFF
    ) {
        return
            calculate(
                currentPose,
                targetPose,
                targetPose.getRotation(),
                xFF,
                yFF,
                thetaFF);
    }

    public ChassisSpeeds calculate(
        Pose2d currentPose,
        Pose2d targetPose,
        Rotation2d targetAngle,
        double driveVelocityFF,
        double thetaFF
    ) {
        return
            calculate(
                currentPose,
                targetPose,
                targetAngle,
                driveVelocityFF * targetPose.getRotation().getCos(),
                driveVelocityFF * targetPose.getRotation().getSin(),
                thetaFF);
    }

    public ChassisSpeeds calculate(Pose2d currentPose, HolonomicState state) {
        return
            calculate(
                currentPose,
                state.poseMeters(),
                state.holonomicAngle(),
                state.velocityMetersPerSecond(),
                state.angularVelocityRadiansPerSec());
    }
}
