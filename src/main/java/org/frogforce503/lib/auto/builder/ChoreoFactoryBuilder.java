package org.frogforce503.lib.auto.builder;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import org.frogforce503.lib.swerve.SwervePathFollower;
import org.frogforce503.robot2025.Constants;
import org.frogforce503.robot2025.subsystems.drive.Drive;
import org.frogforce503.robot2025.subsystems.drive.DriveConstants;

public class ChoreoFactoryBuilder {
    private final Drive drive;
    private final SwervePathFollower pathFollower;

    public ChoreoFactoryBuilder(Drive drive) {
        this.drive = drive;
        this.pathFollower = DriveConstants.pathFollower;
    }

    public AutoFactory buildFactory() {
        return
            new AutoFactory(
                drive::getCurrentPose,
                drive::setPose,
                this::followChoreoTrajectory,
                Constants.useAllianceFlipping,
                drive);
    }
    
    private void followChoreoTrajectory(SwerveSample sample) {
        // Generate the next robot-relative speeds for the robot
        ChassisSpeeds speeds =
            pathFollower.calculate(
                drive.getCurrentPose(),
                sample.getPose(),
                sample.vx,
                sample.vy,
                sample.omega);

        // Apply the generated speeds (with module forces)
        drive.runVelocity(
            speeds,
            sample.moduleForcesX(),
            sample.moduleForcesY());
    }
}