package org.frogforce503.robot2025.commands.drive;

import java.util.function.Supplier;

import org.frogforce503.lib.planning.planned_path.PlannedPath;
import org.frogforce503.lib.swerve.SwervePathFollower;
import org.frogforce503.robot2025.FieldInfo;
import org.frogforce503.robot2025.subsystems.drive.Drive;
import org.frogforce503.robot2025.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import lombok.Setter;

public class DrivePlannedPath extends Command {
    // Requirements
    private final Drive drive;

    // Control
    private final SwervePathFollower controller = DriveConstants.pathFollower;
    private final Timer timer;
    private final Supplier<PlannedPath> dynamicPath;

    // State
    private final boolean willStopAtEnd;
    private double lastTime = 0;
    private Translation2d lastPosition = Translation2d.kZero;

    // Overrides
    @Setter private Supplier<Rotation2d> headingOverride = null;

    public DrivePlannedPath(Drive drive, Supplier<PlannedPath> dynamicPath) {
        this.drive = drive;

        this.controller.setPoseTolerance(
            new Pose2d(
                new Translation2d(Units.inchesToMeters(0.1), Units.inchesToMeters(0.0254)),
                Rotation2d.fromDegrees(1)));
        this.timer = new Timer();
        this.dynamicPath = dynamicPath;

        this.willStopAtEnd =
            dynamicPath
                .get()
                .getDriveTrajectory()
                .sample(dynamicPath.get().getTotalTimeSeconds())
                .velocityMetersPerSecond == 0.00; // originally 0.1

        addRequirements(drive);
    }

    public DrivePlannedPath(Drive drive, PlannedPath path) {
        this(drive, () -> path);
    }

    @Override
    public void initialize() {       
        PlannedPath path = dynamicPath.get(); 

        timer.reset();
        controller.reset();
        timer.start();

        lastPosition = path.getInitialHolonomicPose().getTranslation();
        lastTime = 0;

        var poses =
            dynamicPath
                .get()
                .getDriveTrajectory()
                .getStates()
                .stream()
                .map(state -> state.poseMeters)
                .toArray(Pose2d[]::new);

        FieldInfo
            .getObject("CurrentTrajectory")
            .setPoses(poses);
    }

    @Override
    public void execute() {
        // Get inputs
        PlannedPath path = dynamicPath.get();
        double currentTime = timer.get();
        Pose2d currentPose = drive.getCurrentPose();

        PlannedPath.HolonomicState desiredState = path.sample(currentTime);

        if (headingOverride != null) {
            desiredState
                .withNewHolonomicAngle(headingOverride.get());
        }
        
        // Calculate speeds
        ChassisSpeeds targetChassisSpeeds = controller.calculate(currentPose, desiredState);

        Translation2d measuredVelocity =
            currentPose
                .getTranslation()
                .minus(lastPosition)
                .div(currentTime - lastTime);

        // Apply speeds
        drive.runVelocity(targetChassisSpeeds);

        lastTime = currentTime;
        lastPosition = currentPose.getTranslation();

        // Log inputs & outputs
        Logger.recordOutput("FollowPlannedPath/Timestamp", currentTime);

        Logger.recordOutput("FollowPlannedPath/Current Pose", currentPose);
        Logger.recordOutput("FollowPlannedPath/Desired Pose", desiredState.poseMeters());

        Logger.recordOutput("FollowPlannedPath/Current Angle", drive.getAngle());
        Logger.recordOutput("FollowPlannedPath/Desired Angle", desiredState.holonomicAngle());
        
        Logger.recordOutput("FollowPlannedPath/Current Velocity", measuredVelocity);
        Logger.recordOutput("FollowPlannedPath/Desired Velocity", new Translation2d(targetChassisSpeeds.vxMetersPerSecond, targetChassisSpeeds.vyMetersPerSecond));

        Logger.recordOutput("FollowPlannedPath/Drive Error", controller.getPoseError().getTranslation());
        Logger.recordOutput("FollowPlannedPath/Theta Error", controller.getRotationError());

        Logger.recordOutput("FollowPlannedPath/IsFinished", isFinished());
    }

    @Override
    public boolean isFinished() {
        PlannedPath path = dynamicPath.get();

        boolean timeHasFinished = timer.hasElapsed(path.getTotalTimeSeconds());
        boolean poseTolerance = controller.atReference();
        boolean tooLong = timeHasFinished && timer.hasElapsed(path.getTotalTimeSeconds() + 0.5);

        return (timeHasFinished && poseTolerance) || tooLong;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("END " + interrupted);

        FieldInfo.getObject("CurrentTrajectory").setPoses();

        if (willStopAtEnd) {
            drive.stop();
        }
    }
}