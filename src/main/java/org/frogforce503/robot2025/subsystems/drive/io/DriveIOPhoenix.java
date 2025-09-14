package org.frogforce503.robot2025.subsystems.drive.io;

import org.frogforce503.lib.swerve.SwerveDriveCoast;
import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.subsystems.drive.DriveConstants;
import org.frogforce503.robot2025.subsystems.drive.DriveConstants.ModuleName;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
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
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class DriveIOPhoenix extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements DriveIO {
    // Signals
    private final ModuleSignals[] moduleSignals = new ModuleSignals[4];
    private final StatusSignal<Angle> rawGyroYaw;

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
            Robot.bot.phoenixConstants,
            Robot.bot.frontLeftConstants, Robot.bot.frontRightConstants, Robot.bot.backLeftConstants, Robot.bot.backRightConstants);

        rawGyroYaw = super.getPigeon2().getYaw();

        for (int i = 0; i < moduleSignals.length; i++) {
            SwerveModule<TalonFX, TalonFX, CANcoder> module = super.getModule(i);
        
            final TalonFX driveMotor = module.getDriveMotor();
            final TalonFX steerMotor = module.getSteerMotor();
            final CANcoder steerEncoder = module.getEncoder();

            moduleSignals[i] =
                new ModuleSignals(
                    // Inputs from drive motor
                    driveMotor.getPosition(),
                    driveMotor.getVelocity(),
                    driveMotor.getMotorVoltage(),
                    driveMotor.getSupplyCurrent(),
                    driveMotor.getTorqueCurrent(),

                    // Inputs from turn motor
                    steerEncoder.getAbsolutePosition(),
                    steerMotor.getPosition(),
                    steerMotor.getVelocity(),
                    steerMotor.getMotorVoltage(),
                    steerMotor.getSupplyCurrent(),
                    steerMotor.getTorqueCurrent());
        }
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        // Refresh all signals
        BaseStatusSignal.refreshAll(rawGyroYaw);

        for (ModuleSignals data : moduleSignals) {
            BaseStatusSignal.refreshAll(
                // Inputs from drive motor
                data.drivePosition,
                data.driveVelocity,
                data.driveAppliedVolts,
                data.driveSupplyCurrentAmps,
                data.driveTorqueCurrentAmps,

                // Inputs from turn motor
                data.turnPosition,
                data.turnAbsolutePosition,
                data.turnVelocity,
                data.turnAppliedVolts,
                data.turnSupplyCurrentAmps,
                data.turnTorqueCurrentAmps
            );
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
    public ModuleIOData getModuleData(ModuleName moduleName) {
        ModuleSignals signals = moduleSignals[moduleName.moduleIndex];

        return
            new ModuleIOData(
                // Inputs from drive motor
                BaseStatusSignal.isAllGood(
                    signals.drivePosition,
                    signals.driveVelocity,
                    signals.driveAppliedVolts,
                    signals.driveSupplyCurrentAmps,
                    signals.driveTorqueCurrentAmps),
                Units.rotationsToRadians(signals.drivePosition.getValueAsDouble()),
                Units.rotationsToRadians(signals.driveVelocity.getValueAsDouble()),
                signals.driveAppliedVolts.getValueAsDouble(),
                signals.driveSupplyCurrentAmps.getValueAsDouble(),
                signals.driveTorqueCurrentAmps.getValueAsDouble(),

                // Inputs from turn motor
                BaseStatusSignal.isAllGood(
                    signals.turnPosition,
                    signals.turnVelocity,
                    signals.turnAppliedVolts,
                    signals.turnSupplyCurrentAmps,
                    signals.turnTorqueCurrentAmps),
                BaseStatusSignal.isAllGood(signals.turnAbsolutePosition),
                Rotation2d.fromRotations(signals.turnAbsolutePosition.getValueAsDouble()).minus(moduleName.encoderOffset),
                Rotation2d.fromRotations(signals.turnPosition.getValueAsDouble()),
                Units.rotationsToRadians(signals.turnVelocity.getValueAsDouble()),
                signals.turnAppliedVolts.getValueAsDouble(),
                signals.turnSupplyCurrentAmps.getValueAsDouble(),
                signals.turnTorqueCurrentAmps.getValueAsDouble());
    }

    @Override
    public Rotation2d getRawGyroAngle() {
        return Rotation2d.fromDegrees(rawGyroYaw.getValueAsDouble());
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

    private record ModuleSignals(
        // Inputs from drive motor
        StatusSignal<Angle> drivePosition,
        StatusSignal<AngularVelocity> driveVelocity,
        StatusSignal<Voltage> driveAppliedVolts,
        StatusSignal<Current> driveSupplyCurrentAmps,
        StatusSignal<Current> driveTorqueCurrentAmps,

        // Inputs from turn motor
        StatusSignal<Angle> turnAbsolutePosition,
        StatusSignal<Angle> turnPosition,
        StatusSignal<AngularVelocity> turnVelocity,
        StatusSignal<Voltage> turnAppliedVolts,
        StatusSignal<Current> turnSupplyCurrentAmps,
        StatusSignal<Current> turnTorqueCurrentAmps) {}
}
