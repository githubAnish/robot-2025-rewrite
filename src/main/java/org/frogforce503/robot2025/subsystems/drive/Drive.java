package org.frogforce503.robot2025.subsystems.drive;

import static edu.wpi.first.units.Units.Inches;

import org.frogforce503.lib.util.LoggedTracer;
import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.fields.FieldInfo;
import org.frogforce503.robot2025.subsystems.drive.io.DriveIO;
import org.frogforce503.robot2025.subsystems.drive.io.DriveIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;

public class Drive extends SubsystemBase {
    private final DriveIO io;
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

    private final FieldInfo field;

    @Getter private Pose2d currentPose = Pose2d.kZero;
    @Getter private ChassisSpeeds currentVelocity = new ChassisSpeeds();

    @Getter private boolean slowModeEnabled = false;
    @Getter private boolean robotRelative = false;

    @Setter @Getter private boolean coastAfterAutoEnd = false;

    private SwerveVisualizer visualizer;

    private ChassisSpeeds requestedSpeeds = new ChassisSpeeds();

    public Drive(DriveIO io, FieldInfo field) {
        this.io = io;
        this.field = field;

        this.visualizer = new SwerveVisualizer(DriveConstants.FAST_TRANSLATION_METERS_PER_SECOND);

        setPose(Pose2d.kZero);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive", inputs);
        
        currentPose = inputs.data.poseMeters();
        currentVelocity = inputs.data.velocityMeters();

        this.outputTelemetry();

        // Record cycle time
        LoggedTracer.record("Drive");
    }

    private void outputTelemetry() {
        // Toggles
        Logger.recordOutput("Swerve/Toggles/SlowModeEnabled", slowModeEnabled);
        Logger.recordOutput("Swerve/Toggles/RobotRelative", robotRelative);

        // Inputs
        Logger.recordOutput("Swerve/Inputs/Pose", currentPose);
        Logger.recordOutput("Swerve/Inputs/Velocity", currentVelocity);
        Logger.recordOutput("Swerve/Inputs/Velocity/Magnitude", Math.hypot(currentVelocity.vxMetersPerSecond, currentVelocity.vyMetersPerSecond));

        // Status
        Logger.recordOutput("Swerve/State/AttainedWheelSpeed", Units.metersToFeet(inputs.data.state().ModuleStates[0].speedMetersPerSecond));
        Logger.recordOutput("Swerve/State/Current Speeds", requestedSpeeds.toString());

        SwerveModuleState[] states = inputs.data.state().ModuleStates;
        Logger.recordOutput("Swerve/State/ModuleStates", states);

        for (int i = 0; i < states.length; i++) {
            SwerveModuleState state = states[i];

            Logger.recordOutput("Swerve/Module/" + DriveConstants.moduleNames[i] + "/Angle", state.angle.getDegrees());
            Logger.recordOutput("Swerve/Module/" + DriveConstants.moduleNames[i] + "/Velocity", state.speedMetersPerSecond);
        }

        visualizer.updateModules(states, getAngle());
        visualizer.displayModulePoses(currentPose.getTranslation(), getAngle());

        // Field
        Logger.recordOutput("Alliance Color", field.getAlliance());
        Logger.recordOutput("Current Global Pose", currentPose);
        field.setRobotPose(currentPose);
    }

    public void toggleSlowMode() {
        slowModeEnabled = !slowModeEnabled;
    }

    public void toggleRobotRelative() {
        robotRelative = !robotRelative;
    }

    public void setPose(Pose2d pose) {
        io.setPose(pose);
    }

    public void setAngle(Rotation2d rotation) {
        io.setAngle(rotation);
    }

    public void resetRotation() {
        setAngle(
            field.onRedAlliance()
                ? Rotation2d.kZero
                : Rotation2d.kPi);
    }
    
    // Adding vision measurements
    public void acceptVisionMeasurement(EstimatedRobotPose visionPose, Matrix<N3, N1> stdDevs) {
        io.acceptVisionMeasurement(
            visionPose.estimatedPose.toPose2d(),
            visionPose.timestampSeconds,
            stdDevs);
    }

    public void acceptVisionMeasurement(EstimatedRobotPose visionPose) {
        acceptVisionMeasurement(
            visionPose,
            DriveConstants.stdDevs);
    }

    public Rotation2d getAngle() {
        return this.currentPose.getRotation();
    }

    public Rotation2d getGyroRotation() {
        return inputs.data.rawGyroAngle();
    }

    public ChassisSpeeds getFieldVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(currentVelocity, getAngle());
    }

    public void brake() {
        io.brake();
    }

    public void coast() {
        io.coast();
    }

    /** Runs a robot-relative ChassisSpeeds to the drivetrain. */
    public void runVelocity(ChassisSpeeds speeds) {
        io.runVelocity(speeds);
        this.requestedSpeeds = speeds;
    }

    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    public void runCharacterization(double output) {
        io.runCharacterization(output);
    }

    /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output +=
                Units.radiansToRotations(
                    inputs.data.state().ModuleStates[0].speedMetersPerSecond) / 4.0;
        }
        return output;
    }

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] =
                Units.radiansToRotations(
                    inputs.data.state().ModulePositions[i].distanceMeters / Robot.bot.kWheelRadius.in(Inches));
                // This conversion is to get the position of the drive talon (its default units are rotations)
        }
        return values;
    }
}