package org.frogforce503.lib.commands;

import java.util.function.Supplier;

import org.frogforce503.lib.auto.follower.SwervePathFollower;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.fields.FieldInfo;
import org.frogforce503.robot2025.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveFollowPathCommand extends Command {
    private final Drive drive;
    private final FieldInfo field;

    private final SwervePathFollower controller;
    private final Timer timer;

    private final Supplier<PlannedPath> dynamicPath;
    private PlannedPath path;

    private final boolean willStopAtEnd;

    private double lastTime = 0;
    private Rotation2d lastAngle = Rotation2d.kZero;
    private Translation2d lastPosition = new Translation2d();

    private Supplier<Rotation2d> headingOverride = null;

    private double seek = 0;

    DoubleLogEntry timeLog, xLog, yLog, vxLog, vyLog, desiredXLog, desiredYLog, desiredVxLog, desiredVyLog;

    public SwerveFollowPathCommand(Drive drive, FieldInfo field, Supplier<PlannedPath> dynamicPath) {
        this.drive = drive;
        this.field = field;

        this.dynamicPath = dynamicPath;

        this.controller =
            new SwervePathFollower(
                Robot.bot.autoXController, Robot.bot.autoYController, Robot.bot.autoThetaController);

        //originally 1.75 tolerance for x and y
        this.controller.setTolerance(
            new Pose2d(
                new Translation2d(Units.inchesToMeters(0.1), Units.inchesToMeters(0.0254)),
                Rotation2d.fromDegrees(1)));
        
        this.timer = new Timer();
        
        this.willStopAtEnd =
            this.dynamicPath
                .get()
                .getDriveTrajectory()
                .sample(
                    this.dynamicPath
                        .get()
                        .getTotalTimeSeconds())
                .velocityMetersPerSecond == 0.00; //originally 0.1

        addRequirements(drive);
    }

    public SwerveFollowPathCommand(Drive drive, FieldInfo field, PlannedPath path, double seek) {
        this(drive, field, () -> path);
        this.seek = seek;
    }

    public SwerveFollowPathCommand(Drive drive, FieldInfo field, PlannedPath path) {
        this(drive, field, () -> path);
    }

    public SwerveFollowPathCommand setHeadingOverride(Supplier<Rotation2d> rotationSupplier) {
        this.headingOverride = rotationSupplier;
        return this;
    }

    @Override
    public void initialize() {       
        this.path = dynamicPath.get(); 
        this.timer.reset();
        this.controller.reset();
        this.timer.start();

        lastPosition = this.path.getInitialHolonomicPose().getTranslation();
        lastAngle = this.path.getInitialHolonomicPose().getRotation();
        lastTime = 0;

        var poses =
            this.dynamicPath.get()
                .getDriveTrajectory()
                .getStates()
                .stream()
                    .map(state -> (state.poseMeters))
                .toArray(Pose2d[]::new);

        field.getObject("CurrentTrajectory").setPoses(poses);
    }

    @Override
    public void execute() {
        // Get inputs
        this.path = dynamicPath.get();
        double currentTime = this.timer.get() + this.seek;
        Pose2d currentPose = drive.getCurrentPose();

        PlannedPath.HolonomicState desiredState = this.path.sample(currentTime);

        if (headingOverride != null) {
            desiredState
                .withNewHolonomicAngle(headingOverride.get());
        }
        
        // Calculate speeds
        ChassisSpeeds targetChassisSpeeds =
            this.controller
                .calculate(currentPose, desiredState);

        Translation2d measuredVelocity =
            currentPose
                .getTranslation()
                .minus(lastPosition)
                .div(currentTime - lastTime);

        // Apply speeds
        drive.runVelocity(targetChassisSpeeds);

        lastAngle = desiredState.holonomicAngle(); 
        lastTime = currentTime;
        lastPosition = currentPose.getTranslation();

        // Log inputs & outputs
        Logger.recordOutput("SwerveFollowPathCommand/Timestamp", currentTime);

        Logger.recordOutput("SwerveFollowPathCommand/Current Pose", currentPose);
        Logger.recordOutput("SwerveFollowPathCommand/Desired Pose", desiredState.poseMeters());

        Logger.recordOutput("SwerveFollowPathCommand/Current Angle", drive.getAngle());
        Logger.recordOutput("SwerveFollowPathCommand/Desired Angle", desiredState.holonomicAngle());
        
        Logger.recordOutput("SwerveFollowPathCommand/Current Velocity", measuredVelocity);
        Logger.recordOutput("SwerveFollowPathCommand/Desired Velocity", targetChassisSpeeds);

        Logger.recordOutput("SwerveFollowPathCommand/Drive Error", currentPose.getTranslation().getDistance(desiredState.poseMeters().getTranslation()));
        Logger.recordOutput("SwerveFollowPathCommand/Theta Follower Error", currentPose.getRotation().minus(desiredState.holonomicAngle()));
    }

    @Override
    public boolean isFinished() {
        boolean timeHasFinished = this.timer.hasElapsed(this.path.getTotalTimeSeconds());
        boolean poseTolerance = this.controller.atReference();

        boolean tooLong = timeHasFinished && this.timer.hasElapsed(this.path.getTotalTimeSeconds() + 0.5);

        boolean m_isFinished = (timeHasFinished && poseTolerance) || tooLong;
        SmartDashboard.putBoolean("Path Has Finished", m_isFinished);

        return m_isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("END " + interrupted);

        Logger.recordOutput("Swerve/CurrentTrajectory", new Pose2d[] {});

        field.getObject("CurrentTrajectory").setPoses();

        if (this.willStopAtEnd) {
            drive.stop();
        }
    }
}