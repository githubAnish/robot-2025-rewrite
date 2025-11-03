package org.frogforce503.robot2025.subsystems.drive.io;

import org.frogforce503.lib.swerve.SwerveDriveCoast;
import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.subsystems.drive.DriveConstants;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;
import com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveTranslation;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class DriveIOPhoenix extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements DriveIO {
    // Signals
    private final StatusSignal<Angle> rawGyroYaw;
    private final CharacterizationSignals[] characterizationSignals = new CharacterizationSignals[4];

    // State
    private ChassisSpeeds currentVelocity;

    // Requests
    private final ApplyRobotSpeeds RUN_CHASSIS_SPEEDS =
        new ApplyRobotSpeeds()
            .withCenterOfRotation(DriveConstants.CENTER_OF_ROTATION)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withDesaturateWheelSpeeds(true);

    private final SysIdSwerveTranslation RUN_CHARACTERIZATION = new SysIdSwerveTranslation();

    public DriveIOPhoenix() {
        super(
            TalonFX::new, TalonFX::new, CANcoder::new,
            Robot.bot.getDriveConfig().drivetrainConstants(),
            Robot.bot.getDriveConfig().frontLeft(), Robot.bot.getDriveConfig().frontRight(), Robot.bot.getDriveConfig().backLeft(), Robot.bot.getDriveConfig().backRight());

        rawGyroYaw = super.getPigeon2().getYaw();

        for (int i = 0; i < characterizationSignals.length; i++) {
            TalonFX driveMotor = super.getModule(i).getDriveMotor();

            characterizationSignals[i] =
                new CharacterizationSignals(
                    driveMotor.getPosition(),
                    driveMotor.getVelocity());
        }
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        // Refresh all signals
        BaseStatusSignal.refreshAll(rawGyroYaw);

        for (CharacterizationSignals data : characterizationSignals) {
            BaseStatusSignal.refreshAll(
                data.drivePosition,
                data.driveVelocity);
        }

        // Get chassis state & update drive inputs
        SwerveDriveState currentState = super.getState();
        Pose2d currentPose = currentState.Pose;

        currentVelocity =
            super
                .getKinematics()
                .toChassisSpeeds(currentState.ModuleStates);

        inputs.data =
            new DriveIOData(
                currentState,
                currentPose,
                currentVelocity);
    }

    @Override
    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(rawGyroYaw.getValueAsDouble());
    }

    @Override
    public CharacterizationIOData getCharacterizationData(int moduleIndex) {
        CharacterizationSignals signals = characterizationSignals[moduleIndex];

        return
            new CharacterizationIOData(
                Units.rotationsToRadians(signals.drivePosition.getValueAsDouble()),
                Units.rotationsToRadians(signals.driveVelocity.getValueAsDouble()));
    }

    @Override
    public void setPose(Pose2d pose) {
        System.out.println("Setting pose to " + pose);
        super.resetPose(pose);
    }

    @Override
    public void setAngle(Rotation2d angle) {
        System.out.println("Setting angle to " + angle);
        super.resetRotation(angle);
    }

    @Override
    public void acceptVisionMeasurement(Pose2d poseEstimate, double timestamp, Matrix<N3, N1> stdDevs) {
        double newTimestamp = Utils.fpgaToCurrentTime(timestamp);

        if (stdDevs != null) {
            super.addVisionMeasurement(poseEstimate, newTimestamp, stdDevs);
        } else {
            super.addVisionMeasurement(poseEstimate, newTimestamp);
        }
    }

    @Override
    public void brake() {
        super.setControl(new SwerveDriveBrake());
    }

    @Override
    public void coast() {
        super.setControl(new SwerveDriveCoast());
    }

    @Override
    public void runVelocity(ChassisSpeeds speeds) {
        super.setControl(RUN_CHASSIS_SPEEDS.withSpeeds(speeds));
    }

    @Override
    public void runVelocity(ChassisSpeeds speeds, double[] moduleForcesX, double[] moduleForcesY) {
        super.setControl(
            RUN_CHASSIS_SPEEDS
                .withSpeeds(speeds)
                .withWheelForceFeedforwardsX(moduleForcesX)
                .withWheelForceFeedforwardsY(moduleForcesY));
    }
    

    @Override
    public void runCharacterization(double output) {
        super.setControl(RUN_CHARACTERIZATION.withVolts(output));
    }

    private record CharacterizationSignals(
        StatusSignal<Angle> drivePosition,
        StatusSignal<AngularVelocity> driveVelocity) {}
}
