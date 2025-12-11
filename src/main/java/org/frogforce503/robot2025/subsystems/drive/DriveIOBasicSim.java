package org.frogforce503.robot2025.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import lombok.Setter;

public class DriveIOBasicSim implements DriveIO {
    private final SwerveDriveKinematics kinematics;

    // Constants
    private final double theoreticalWheelRadiusInches = 2.00; // Inches

    // State
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

    // Characterization Data
    private double[] wheelPositionRad = new double[4];
    private double[] wheelVelocityRadPerSec = new double[4];

    public DriveIOBasicSim() {
        this.kinematics = DriveConstants.kinematics;
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        SwerveDriveState currentState = getCurrentState(currentPose);

        inputs.data =
            new DriveIOData(
                currentState,
                currentPose,
                currentVelocity);

        update();
    }

    @Override
    public void setPose(Pose2d pose) {
        this.currentPose = pose;
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
    public void runVelocity(ChassisSpeeds speeds) {
        this.currentVelocity = speeds;
        this.states = kinematics.toSwerveModuleStates(this.currentVelocity);
    }

    @Override
    public void runVelocity(ChassisSpeeds speeds, double[] moduleForcesX, double[] moduleForcesY) {
        runVelocity(speeds); // Ignore module force feedforwards in simulation
    }

    public CharacterizationIOData getCharacterizationData(int moduleIndex) {
        return
            new CharacterizationIOData(
                wheelPositionRad[moduleIndex],
                wheelVelocityRadPerSec[moduleIndex]);
    }

    public Rotation2d getGyroYaw() {
        return currentPose.getRotation();
    }

    public void update() {
        double t = Timer.getFPGATimestamp();

        if (lastUpdate > 0) {
            dt = t - lastUpdate;

            currentPose =
                currentPose.exp(
                    new Twist2d(
                        currentVelocity.vxMetersPerSecond * dt, 
                        currentVelocity.vyMetersPerSecond * dt, 
                        currentVelocity.omegaRadiansPerSecond * dt));
        }
        
        lastUpdate = t;

        calculateCharacterizationData();
    }

    private SwerveDriveState getCurrentState(Pose2d currentPose) {
        SwerveDriveState currentState = new SwerveDriveState();
        
        currentState.SuccessfulDaqs = 0;
        currentState.FailedDaqs = 0;
        currentState.Pose = currentPose;
        currentState.ModuleStates = this.states;
        currentState.OdometryPeriod = 0.02;

        return currentState;
    }

    private void calculateCharacterizationData() {
        for (int i = 0; i < 4; i++) {
            double wheelSpeedMetersPerSec = this.states[i].speedMetersPerSecond;
            wheelVelocityRadPerSec[i] = wheelSpeedMetersPerSec / theoreticalWheelRadiusInches;

            wheelPositionRad[i] += wheelVelocityRadPerSec[i] * dt;
        }
    }    
}
