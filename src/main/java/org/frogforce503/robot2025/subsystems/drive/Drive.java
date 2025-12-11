package org.frogforce503.robot2025.subsystems.drive;

import org.frogforce503.lib.logging.LoggedTracer;
import org.frogforce503.lib.vision.apriltag_detection.VisionMeasurement;
import org.frogforce503.robot2025.FieldInfo;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;

public class Drive extends SubsystemBase {
    private final DriveIO io;
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

    // State
    private ChassisSpeeds requestedSpeeds = new ChassisSpeeds();

    // Toggles
    @Getter private boolean robotRelative = false;
    @Getter private boolean slowMode = false;
    @Setter @Getter private boolean coastAfterAutoEnd = false;

    public Drive(DriveIO io) {
        this.io = io;

        setPose(Pose2d.kZero);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive", inputs);

        this.outputTelemetry();

        // Record cycle time
        LoggedTracer.record("Drive");
    }

    private void outputTelemetry() {
        // Toggles
        Logger.recordOutput("Drive/Toggles/SlowModeEnabled", slowMode);
        Logger.recordOutput("Drive/Toggles/RobotRelative", robotRelative);

        // Inputs
        Logger.recordOutput("Drive/Inputs/Pose", getCurrentPose());
        Logger.recordOutput("Drive/Inputs/Velocity", getCurrentVelocity());
        Logger.recordOutput("Drive/Inputs/Velocity/Magnitude", Math.hypot(getCurrentVelocity().vxMetersPerSecond, getCurrentVelocity().vyMetersPerSecond));

        // Status
        Logger.recordOutput("Drive/State/AttainedWheelSpeed", Units.metersToFeet(inputs.data.state().ModuleStates[0].speedMetersPerSecond));
        Logger.recordOutput("Drive/State/Current Speeds", requestedSpeeds);

        SwerveModuleState[] states = inputs.data.state().ModuleStates;
        Logger.recordOutput("Drive/State/ModuleStates", states);

        for (int i = 0; i < states.length; i++) {
            SwerveModuleState state = states[i];

            Logger.recordOutput("Drive/Module/" + ModuleName.values()[i] + "/Angle", state.angle.getDegrees());
            Logger.recordOutput("Drive/Module/" + ModuleName.values()[i] + "/Velocity", state.speedMetersPerSecond);
        }

        // Field
        Logger.recordOutput("Current Global Pose", getCurrentPose());
        FieldInfo.setRobotPose(getCurrentPose());
    }

    // Toggles
    public void toggleSlowMode() {
        slowMode = !slowMode;
    }

    public void toggleRobotRelative() {
        robotRelative = !robotRelative;
    }

    // Setters
    public void setPose(Pose2d pose) {
        io.setPose(pose);
    }

    public void setAngle(Rotation2d rotation) {
        io.setAngle(rotation);
    }

    public void resetRotation() {
        setAngle(
            FieldInfo.isRed()
                ? Rotation2d.kZero
                : Rotation2d.kPi);
    }
    
    // Adding vision measurements
    public void acceptVisionMeasurement(VisionMeasurement measurement) {
        io.acceptVisionMeasurement(
            measurement.pose(),
            measurement.timestamp(),
            measurement.standardDeviations());
    }

    // Getters
    public Pose2d getCurrentPose() {
        return inputs.data.poseMeters();
    }

    public ChassisSpeeds getCurrentVelocity() {
        return inputs.data.velocityMeters();
    }

    public Rotation2d getAngle() {
        return getCurrentPose().getRotation();
    }

    public ChassisSpeeds getFieldVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getCurrentVelocity(), getAngle());
    }

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = io.getCharacterizationData(i).drivePositionRad();
        }
        return values;
    }

    /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output +=
                Units.radiansToRotations(io.getCharacterizationData(i).driveVelocityRadPerSec()) / 4.0;
        }
        return output;
    }

    public Rotation2d getGyroRotation() {
        return io.getGyroYaw();
    }

    // Actions
    public void coast() {
        io.coast();
    }

    public void brake() {
        io.brake();
    }

    /** Runs a robot-relative ChassisSpeeds to the drivetrain. */
    public void runVelocity(ChassisSpeeds speeds) {
        io.runVelocity(speeds);
        this.requestedSpeeds = speeds;
    }

    /** Runs a robot-relative ChassisSpeeds to the drivetrain with wheel force feedforwards in the X & Y direction. */
    public void runVelocity(ChassisSpeeds speeds, double[] moduleForcesX, double[] moduleForcesY) {
        io.runVelocity(speeds, moduleForcesX, moduleForcesY);
        this.requestedSpeeds = speeds;
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        io.runCharacterization(output);
    }

    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    private enum ModuleName {
        FrontLeft,
        FrontRight,
        BackLeft,
        BackRight
    }
}