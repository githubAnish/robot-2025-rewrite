package org.frogforce503.robot2025.subsystems.drive.io;

import org.frogforce503.robot2025.Robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import lombok.Setter;

public class DriveIOSim implements DriveIO {
    private SwerveDriveKinematics kinematics;

    private SwerveModuleState[] states =
        new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()};

    @Setter private Pose2d currentPose = Pose2d.kZero;
    private ChassisSpeeds currentVelocity = new ChassisSpeeds();

    private double lastUpdate = -1.0;
    private double dt = 0;

    public DriveIOSim() {
        kinematics = Robot.bot.kinematics;
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        SwerveDriveState currentState = getCurrentState(currentPose);

        inputs.data =
            new DriveIOData(
                currentState,
                currentPose,
                currentVelocity,
                currentPose.getRotation());

        update();
    }

    @Override
    public void setAngle(Rotation2d angle) {
        this.currentPose = new Pose2d(this.currentPose.getTranslation(), angle);
    }

    @Override
    public void brake() {
        this.currentVelocity = new ChassisSpeeds();
    }

    @Override
    public void runVelocity(ChassisSpeeds velocity) {
        this.currentVelocity = velocity;
        this.states = kinematics.toSwerveModuleStates(this.currentVelocity);
    }

    public void update() {
        double t = Timer.getFPGATimestamp();
        if (lastUpdate > 0) {
            dt = t - lastUpdate;
            currentPose = currentPose.exp(
                new Twist2d(
                    currentVelocity.vxMetersPerSecond * dt, 
                    currentVelocity.vyMetersPerSecond * dt, 
                    currentVelocity.omegaRadiansPerSecond * dt
                )
            );
        }
        lastUpdate = t;
    }

    public SwerveDriveState getCurrentState(Pose2d nowPose) {
        SwerveDriveState currentState = new SwerveDriveState();
        
        currentState.SuccessfulDaqs = 0;
        currentState.FailedDaqs = 0;
        currentState.Pose = nowPose;
        currentState.ModuleStates = this.states;
        currentState.OdometryPeriod = 0.02;

        return currentState;
    }
}
