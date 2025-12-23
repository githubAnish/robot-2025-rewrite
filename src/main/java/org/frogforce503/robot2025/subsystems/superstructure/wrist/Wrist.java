package org.frogforce503.robot2025.subsystems.superstructure.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.RobotState;
import lombok.Getter;
import lombok.Setter;

import org.frogforce503.lib.logging.LoggedTracer;
import org.frogforce503.lib.subsystem.FFSubsystemBase;
import org.frogforce503.robot2025.Constants;
import org.frogforce503.robot2025.Robot;
import org.littletonrobotics.junction.Logger;

public class Wrist extends FFSubsystemBase {
    private final WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    // Constants
    @Setter private ArmFeedforward feedforward;

    // Control
    private double targetAngleRad = WristConstants.START;
    
    private boolean shouldRunProfile = true;
    @Setter private TrapezoidProfile profile;
    @Getter private State setpoint = new State();
    private boolean atGoal = false;

    public Wrist(WristIO io) {
        this.io = io;

        feedforward = Robot.bot.getWristConfig().kFF().getArmFF();
        profile = new TrapezoidProfile(Robot.bot.getWristConfig().kConstraints());
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs("Wrist", inputs);

        // Update profile
        if (shouldRunProfile && RobotState.isEnabled()) {
            var goalState =
                new State(
                    MathUtil.clamp(targetAngleRad, WristConstants.minAngle, WristConstants.maxAngle),
                    0.0);

            double previousVelocity = setpoint.velocity;

            setpoint = profile.calculate(Constants.loopPeriodSecs, setpoint, goalState);
            atGoal = isAtAngle(goalState.position, WristConstants.kTolerance);

            double accel = (setpoint.velocity - previousVelocity) / Constants.loopPeriodSecs;
            io.runPosition(setpoint.position, feedforward.calculate(setpoint.position, setpoint.velocity, accel));

            // Log state
            Logger.recordOutput("Wrist/Profile/SetpointPositionRad", setpoint.position);
            Logger.recordOutput("Wrist/Profile/SetpointVelocityRadPerSec", setpoint.velocity);
            Logger.recordOutput("Wrist/Profile/GoalPositionRad", goalState.position);
            Logger.recordOutput("Wrist/AtGoal", atGoal);
        } else {
            // Reset setpoint
            setpoint = new State(getRelativeAngleRad(), 0.0);
      
            // Clear logs
            Logger.recordOutput("Wrist/Profile/SetpointPositionRad", 0.0);
            Logger.recordOutput("Wrist/Profile/SetpointVelocityRadPerSec", 0.0);
            Logger.recordOutput("Wrist/Profile/GoalPositionRad", 0.0);
            Logger.recordOutput("Wrist/AtGoal", true);
        }

        Logger.recordOutput("Wrist/CurrentRelativePositionRad", getRelativeAngleRad());
        Logger.recordOutput("Wrist/CurrentAbsolutePositionRad", getAbsoluteAngleRad());

        // Record cycle time
        LoggedTracer.record("Wrist");
    }

    public double getRelativeAngleRad() {
        return inputs.data.relativePositionRad();
    }

    public double getAbsoluteAngleRad() {
        return inputs.data.absolutePositionRad();
    }

    // Actions
    public void setRelativeEncoderPosition(double position) {
        io.setRelativeEncoderPosition(position);
    }

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
        return MathUtil.isNear(angleRad, getRelativeAngleRad(), tolerance);
    }
}