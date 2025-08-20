package org.frogforce503.robot2025.subsystems.drive.io;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface DriveIO {
    @AutoLog
    class DriveIOInputs {
        public DriveIOData data =
            new DriveIOData(
                new SwerveDriveState(),
                Pose2d.kZero,
                new ChassisSpeeds());
    }

    record DriveIOData(
        SwerveDriveState state,
        Pose2d poseMeters,
        ChassisSpeeds velocityMeters) {}

    record ModuleIOData(
        boolean driveConnected,
        double drivePositionRad,
        double driveVelocityRadPerSec,
        double driveAppliedVolts,
        double driveSupplyCurrentAmps,
        double driveTorqueCurrentAmps,
        boolean turnConnected,
        boolean turnEncoderConnected,
        Rotation2d turnAbsolutePosition,
        Rotation2d turnPosition,
        double turnVelocityRadPerSec,
        double turnAppliedVolts,
        double turnSupplyCurrentAmps,
        double turnTorqueCurrentAmps
    ) {
        public ModuleIOData() {
            this(
                false,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                false,
                false,
                Rotation2d.kZero,
                Rotation2d.kZero,
                0.0,
                0.0,
                0.0,
                0.0);
        }
    }

    default void updateInputs(DriveIOInputs inputs) {}

    default ModuleIOData getModuleData(int moduleIndex, Rotation2d encoderOffsetForModule) {
        return new ModuleIOData();
    }

    default Rotation2d getRawGyroAngle() {
        return Rotation2d.kZero;
    }

    default void setPose(Pose2d pose) {}

    default void setAngle(Rotation2d angle) {}

    default void acceptVisionMeasurement(Pose2d poseEstimate, double timestamp, Matrix<N3, N1> stdDevs) {}

    default void brake() {}

    default void coast() {}

    default void runVelocity(ChassisSpeeds velocity) {}

    default void runCharacterization(double output) {}
}
