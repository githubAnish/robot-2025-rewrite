package org.frogforce503.robot2025.subsystems.superstructure.elevator;

import org.frogforce503.lib.subsystem.FFSubsystemBase;
import org.frogforce503.lib.util.LoggedTracer;
import org.frogforce503.robot2025.Constants;
import org.frogforce503.robot2025.Robot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.RobotState;
import lombok.Getter;
import lombok.Setter;

public class Elevator extends FFSubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    // Constants
    @Setter private ElevatorFeedforward feedforward;
    
    // Control
    private double targetHeightMeters = ElevatorConstants.START;
    private double lastHeightMeters = 0.0;

    private boolean shouldRunProfile = false;
    @Setter private TrapezoidProfile profile;
    @Getter private State setpoint = new State();
    private boolean atGoal = false;

    public Elevator(ElevatorIO io) {
        this.io = io;
        
        feedforward = Robot.bot.getElevatorConfig().kFF().getElevatorFF();
        profile = new TrapezoidProfile(Robot.bot.getElevatorConfig().kConstraints());
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        // Reset encoder if limit switch pressed & elevator is going down
        if (inputs.data.limitSwitchPressed() && getHeightMeters() < lastHeightMeters) {
            io.resetEncoder();
            setpoint = new State(0.0, 0.0);
        }

        // Update profile
        if (shouldRunProfile && RobotState.isEnabled()) {
            var goalState =
                new State(
                    MathUtil.clamp(targetHeightMeters, ElevatorConstants.minHeight, ElevatorConstants.maxHeight),
                    0.0);

            double previousVelocity = setpoint.velocity;

            setpoint = profile.calculate(Constants.loopPeriodSecs, setpoint, goalState);
            atGoal = isAtHeight(goalState.position, ElevatorConstants.kTolerance);

            double accel = (setpoint.velocity - previousVelocity) / Constants.loopPeriodSecs;
            io.runPosition(setpoint.position, feedforward.calculate(setpoint.velocity, accel));

            /// Log state
            Logger.recordOutput("Elevator/Profile/SetpointPositionMeters", setpoint.position);
            Logger.recordOutput("Elevator/Profile/SetpointVelocityMetersPerSec", setpoint.velocity);
            Logger.recordOutput("Elevator/Profile/GoalPositionMeters", goalState.position);
            Logger.recordOutput("Elevator/AtGoal", atGoal);
        } else {
            // Reset setpoint
            setpoint = new State(getHeightMeters(), 0.0);
      
            // Clear logs
            Logger.recordOutput("Elevator/Profile/SetpointPositionMeters", 0.0);
            Logger.recordOutput("Elevator/Profile/SetpointVelocityMetersPerSec", 0.0);
            Logger.recordOutput("Elevator/Profile/GoalPositionMeters", 0.0);
            Logger.recordOutput("Elevator/AtGoal", true);
        }

        Logger.recordOutput("Elevator/CurrentPositionMeters", getHeightMeters());
        lastHeightMeters = getHeightMeters();

        // Record cycle time
        LoggedTracer.record("Elevator");
    }

    public double getHeightMeters() {
        return inputs.data.positionMeters();
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
        shouldRunProfile = false;

        // Prevent downward motion into the limit switch
        if (inputs.data.limitSwitchPressed() && volts < 0) {
            volts = 0;
        }

        io.runVolts(volts);
    }

    public void setHeight(double heightMeters) {
        this.shouldRunProfile = true;
        this.targetHeightMeters = heightMeters;
    }

    public boolean isAtHeight(double heightMeters, double tolerance) {
        return MathUtil.isNear(heightMeters, getHeightMeters(), tolerance);
    }
}