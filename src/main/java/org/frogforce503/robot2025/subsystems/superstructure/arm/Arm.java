package org.frogforce503.robot2025.subsystems.superstructure.arm;

import org.frogforce503.robot2025.Constants;
import org.frogforce503.robot2025.Robot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.RobotState;
import lombok.Getter;
import lombok.Setter;

import org.frogforce503.lib.logging.LoggedTracer;
import org.frogforce503.lib.subsystem.FFSubsystemBase;

public class Arm extends FFSubsystemBase {
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    // Constants
    @Setter private ArmFeedforward feedforward;

    // Control
    private double targetAngleRad = ArmConstants.START;

    private boolean shouldRunProfile = false;
    @Setter private TrapezoidProfile profile;
    @Getter private State setpoint = new State();
    private boolean atGoal = false;

    public Arm(ArmIO io) {
        this.io = io;

        feedforward = Robot.bot.getArmConfig().kFF().getArmFF();
        profile = new TrapezoidProfile(Robot.bot.getArmConfig().kConstraints());
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        // Update profile
        if (shouldRunProfile && RobotState.isEnabled()) {
            var goalState =
                new State(
                    MathUtil.clamp(targetAngleRad, ArmConstants.minAngle, ArmConstants.maxAngle),
                    0.0);

            double previousVelocity = setpoint.velocity;

            setpoint = profile.calculate(Constants.loopPeriodSecs, setpoint, goalState);
            atGoal = isAtAngle(goalState.position, ArmConstants.kTolerance);

            double accel = (setpoint.velocity - previousVelocity) / Constants.loopPeriodSecs;
            io.runPosition(setpoint.position, feedforward.calculate(setpoint.position, setpoint.velocity, accel));

            // Log state
            Logger.recordOutput("Arm/Profile/SetpointPositionRad", setpoint.position);
            Logger.recordOutput("Arm/Profile/SetpointVelocityRadPerSec", setpoint.velocity);
            Logger.recordOutput("Arm/Profile/GoalPositionRad", goalState.position);
            Logger.recordOutput("Arm/AtGoal", atGoal);
        } else {
            // Reset setpoint
            setpoint = new State(getAngleRad(), 0.0);
      
            // Clear logs
            Logger.recordOutput("Arm/Profile/SetpointPositionRad", 0.0);
            Logger.recordOutput("Arm/Profile/SetpointVelocityRadPerSec", 0.0);
            Logger.recordOutput("Arm/Profile/GoalPositionRad", 0.0);
            Logger.recordOutput("Arm/AtGoal", true);
        }

        Logger.recordOutput("Arm/CurrentPositionRad", getAngleRad());
        Logger.recordOutput("Arm/CurrentVelocityRadPerSec", inputs.data.velocityRadPerSec());
        Logger.recordOutput("Arm/CurrentAppliedVolts", inputs.data.appliedVolts());

        // Record cycle time
        LoggedTracer.record("Arm");
    }

    public double getAngleRad() {
        return inputs.data.positionRad();
    }

    // Actions
    public void setPID(double kP, double kI, double kD) {
        io.setPID(kP, kI, kD);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        io.setBrakeMode(enabled);
    }

    @Override
    public void stop() {
        io.stop();
    }

    public void runVolts(double volts) {
        this.shouldRunProfile = false;
        io.runVolts(volts);
    }

    public void setAngle(double angleRad) {
        this.shouldRunProfile = true;
        this.targetAngleRad = angleRad;
    }

    public boolean isAtAngle(double angleRad, double tolerance) {
        return MathUtil.isNear(angleRad, getAngleRad(), tolerance);
    }
}