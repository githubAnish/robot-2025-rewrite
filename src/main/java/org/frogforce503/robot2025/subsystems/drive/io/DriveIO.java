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
        public DriveIOData data = new DriveIOData(new SwerveDriveState(), new Pose2d(), new ChassisSpeeds());
    }

    record DriveIOData(
        SwerveDriveState state,
        Pose2d poseMeters,
        ChassisSpeeds velocityMeters) {}

    default void updateInputs(DriveIOInputs inputs) {}

    default void setPose(Pose2d pose) {}

    default void setAngle(Rotation2d angle) {}

    default void runVelocity(ChassisSpeeds velocity) {}

    default void brake() {}

    default void coast() {}
    
    default void acceptVisionMeasurement(Pose2d poseEstimate, double timestamp, Matrix<N3, N1> stdDevs) {}
}
