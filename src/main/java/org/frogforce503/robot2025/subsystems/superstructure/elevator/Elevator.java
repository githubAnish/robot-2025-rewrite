package org.frogforce503.robot2025.subsystems.superstructure.elevator;

import org.frogforce503.lib.math.Range;
import org.frogforce503.lib.subsystem.FFSubsystemBase;
import org.frogforce503.lib.util.LoggedTracer;
import org.frogforce503.robot2025.Constants;
import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.subsystems.superstructure.sensors.LimitSwitchIO;
import org.frogforce503.robot2025.subsystems.superstructure.sensors.LimitSwitchIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.RobotState;
import lombok.Getter;
import lombok.Setter;

public class Elevator extends FFSubsystemBase {
    private final ElevatorIO elevatorIO;
    private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
    
    private final LimitSwitchIO limitSwitchIO;
    private final LimitSwitchIOInputsAutoLogged limitSwitchInputs = new LimitSwitchIOInputsAutoLogged();

    // Constants
    private final Range motionRange = Robot.bot.getElevatorConfig().motionRange();
    @Setter private ElevatorFeedforward feedforward = Robot.bot.getElevatorConfig().kFF().getElevatorFF();
    
    // Control
    private double targetHeightMeters = ElevatorConstants.START;

    private boolean shouldRunProfile = false;
    @Setter private TrapezoidProfile profile;
    @Getter private State setpoint = new State();
    private boolean atGoal = false;

    public Elevator(ElevatorIO elevatorIO, LimitSwitchIO limitSwitchIO) {
        this.elevatorIO = elevatorIO;
        this.limitSwitchIO = limitSwitchIO;
        profile = new TrapezoidProfile(Robot.bot.getElevatorConfig().kConstraints());
    }

    @Override
    public void periodic() {
        super.periodic();

        elevatorIO.updateInputs(elevatorInputs);
        Logger.processInputs("Elevator/Elevator", elevatorInputs);

        limitSwitchIO.updateInputs(limitSwitchInputs);
        Logger.processInputs("Elevator/LimitSwitch", limitSwitchInputs);

        // Reset encoder if limit switch pressed & elevator is going down
        if (limitSwitchInputs.data.pressed() && getHeightMeters() > setpoint.position) {
            elevatorIO.resetEncoder();
            setpoint = new State(0.0, 0.0);
        }

        // Update profile
        if (shouldRunProfile && RobotState.isEnabled()) {
            var goalState =
                new State(
                    motionRange.clamp(targetHeightMeters),
                    0.0);

            double previousVelocity = setpoint.velocity;

            setpoint =
                profile
                    .calculate(Constants.loopPeriodSecs, setpoint, goalState);

            if (!motionRange.contains(setpoint.position)) {
                setpoint =
                    new State(
                        motionRange.clamp(setpoint.position),
                        0.0);
            }

            atGoal = isAtHeight(goalState.position, ElevatorConstants.kTolerance);

            if (atGoal) {
                stop();
            } else {
                double accel = (setpoint.velocity - previousVelocity) / Constants.loopPeriodSecs;
                elevatorIO.runPosition(setpoint.position, feedforward.calculate(setpoint.velocity, accel));
            }

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

        // Record cycle time
        LoggedTracer.record("Elevator");
    }

    public double getHeightMeters() {
        return elevatorInputs.data.positionMeters();
    }

    // Actions
    public void setPID(double kP, double kI, double kD) {
        elevatorIO.setPID(kP, kI, kD);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        elevatorIO.setBrakeMode(enabled);
    }

    @Override
    public void stop() {
        elevatorIO.stop();
    }

    public void runOpenLoop(double output) {
        shouldRunProfile = false;
        elevatorIO.runOpenLoop(output);
    }

    public void setHeight(double heightMeters) {
        this.shouldRunProfile = true;
        this.targetHeightMeters = heightMeters;
    }

    public boolean isAtHeight(double heightMeters, double tolerance) {
        return MathUtil.isNear(heightMeters, getHeightMeters(), tolerance);
    }
}