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
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class DriveIOPhoenix extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements DriveIO {
    private ChassisSpeeds currentVelocity;

    private final ApplyRobotSpeeds RUN_CHASSIS_SPEEDS =
        new ApplyRobotSpeeds()
            .withCenterOfRotation(DriveConstants.CENTER_OF_ROTATION)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    private final SysIdSwerveTranslation RUN_CHARACTERIZATION =
        new SysIdSwerveTranslation();

    public DriveIOPhoenix() {
        super(
            TalonFX::new, TalonFX::new, CANcoder::new,
            Robot.bot.phoenixConstants,
            Robot.bot.frontLeftConstants, Robot.bot.frontRightConstants, Robot.bot.backLeftConstants, Robot.bot.backRightConstants);
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
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
    public ModuleIOData getModuleData(int moduleIndex, Rotation2d encoderOffsetForModule) {
        TalonFX driveMotor = super.getModule(moduleIndex).getDriveMotor();
        TalonFX steerMotor = super.getModule(moduleIndex).getSteerMotor();
        CANcoder steerEncoder = super.getModule(moduleIndex).getEncoder();

        // Inputs from drive motor
        final StatusSignal<Angle> drivePosition = driveMotor.getPosition();
        final StatusSignal<AngularVelocity> driveVelocity = driveMotor.getVelocity();
        final StatusSignal<Voltage> driveAppliedVolts = driveMotor.getMotorVoltage();
        final StatusSignal<Current> driveSupplyCurrentAmps = driveMotor.getSupplyCurrent();
        final StatusSignal<Current> driveTorqueCurrentAmps = driveMotor.getTorqueCurrent();

        // Inputs from turn motor
        final StatusSignal<Angle> turnAbsolutePosition = steerEncoder.getAbsolutePosition();
        final StatusSignal<Angle> turnPosition = steerMotor.getPosition();
        final StatusSignal<AngularVelocity> turnVelocity = steerMotor.getVelocity();
        final StatusSignal<Voltage> turnAppliedVolts = steerMotor.getMotorVoltage();
        final StatusSignal<Current> turnSupplyCurrentAmps = steerMotor.getSupplyCurrent();
        final StatusSignal<Current> turnTorqueCurrentAmps = steerMotor.getTorqueCurrent();

        return
            new ModuleIOData(
                BaseStatusSignal.isAllGood(
                    drivePosition,
                    driveVelocity,
                    driveAppliedVolts,
                    driveSupplyCurrentAmps,
                    driveTorqueCurrentAmps),
                Units.rotationsToRadians(drivePosition.getValueAsDouble()),
                Units.rotationsToRadians(driveVelocity.getValueAsDouble()),
                driveAppliedVolts.getValueAsDouble(),
                driveSupplyCurrentAmps.getValueAsDouble(),
                driveTorqueCurrentAmps.getValueAsDouble(),
                BaseStatusSignal.isAllGood(
                    turnPosition,
                    turnVelocity,
                    turnAppliedVolts,
                    turnSupplyCurrentAmps,
                    turnTorqueCurrentAmps),
                BaseStatusSignal.isAllGood(turnAbsolutePosition),
                Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble()).minus(encoderOffsetForModule),
                Rotation2d.fromRotations(turnPosition.getValueAsDouble()),
                Units.rotationsToRadians(turnVelocity.getValueAsDouble()),
                turnAppliedVolts.getValueAsDouble(),
                turnSupplyCurrentAmps.getValueAsDouble(),
                turnTorqueCurrentAmps.getValueAsDouble());
    }

    @Override
    public Rotation2d getRawGyroAngle() {
        return
            Rotation2d.fromDegrees(
                super
                    .getPigeon2()
                    .getYaw()
                    .getValueAsDouble());
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
    public void runCharacterization(double output) {
        super.setControl(RUN_CHARACTERIZATION.withVolts(output));
    }
}
