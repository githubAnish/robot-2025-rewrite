package org.frogforce503.lib.auto.builder;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.subsystems.drive.Drive;

public class ChoreoFactoryBuilder {
    private final Drive drive;

    private final boolean useAllianceFlipping = false;

    public ChoreoFactoryBuilder(Drive drive) {
        this.drive = drive;
    }

    public AutoFactory buildFactory() {
        return
            new AutoFactory(
                drive::getCurrentPose,
                drive::setPose,
                sample ->
                    followChoreoTrajectory(
                        (SwerveSample) sample,
                        Robot.bot.autoPIDController.autoXController(),
                        Robot.bot.autoPIDController.autoYController(),
                        Robot.bot.autoPIDController.autoThetaController()),
                useAllianceFlipping,
                drive);
    }
    
    private void followChoreoTrajectory(SwerveSample sample, PIDController xController, PIDController yController, PIDController thetaController) {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // Get the current pose of the robot
        Pose2d pose = drive.getCurrentPose();

        // Generate the next speeds for the robot
        ChassisSpeeds speeds =
            new ChassisSpeeds(
                sample.vx + xController.calculate(pose.getX(), sample.x),
                sample.vy + yController.calculate(pose.getY(), sample.y),
                sample.omega + thetaController.calculate(pose.getRotation().getRadians(), sample.heading));

        // Apply the generated speeds
        drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                pose.getRotation()));
    }
}