package org.frogforce503.robot2025.subsystems.superstructure.elevator;

import org.frogforce503.lib.motorcontrol.tuning.TuningService;
import org.frogforce503.lib.motorcontrol.tuning.pidf.PIDFConfig;
import org.frogforce503.lib.motorcontrol.tuning.pidf.PIDFTuningService;
import org.frogforce503.lib.motorcontrol.tuning.speed.SpeedConstraintsTuningService;
import org.frogforce503.lib.math.Range;
import org.frogforce503.lib.subsystem.FFSubsystemBase;
import org.frogforce503.lib.util.LoggedTracer;
import org.frogforce503.robot2025.Constants;
import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.subsystems.superstructure.sensors.LimitSwitchIO;
import org.frogforce503.robot2025.subsystems.superstructure.sensors.LimitSwitchIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.function.BooleanConsumer;
import lombok.Getter;

public class Elevator extends FFSubsystemBase {
    private final ElevatorIO elevatorIO;
    private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
    
    private final LimitSwitchIO limitSwitchIO;
    private final LimitSwitchIOInputsAutoLogged limitSwitchInputs = new LimitSwitchIOInputsAutoLogged();

    // Constants
    private final Range range = Robot.bot.getElevatorConfig().range();
    private ElevatorFeedforward feedforward = Robot.bot.getElevatorConfig().kPIDF().toElevatorFF();
    private final double tolerance = 0.5;
    
    // Control
    private double targetHeight = ElevatorGoal.START.getHeight();

    private boolean shouldRunProfile = false;
    private TrapezoidProfile profile;
    @Getter private State setpoint = new State();
    private boolean atGoal = false;

    // Tuning
    private LoggedNetworkBoolean tuningEnabled =
        new LoggedNetworkBoolean("Tuning/Elevator/Tuning?", false);

    private TuningService<PIDFConfig> pidfTuningService =
        new PIDFTuningService("Elevator", Robot.bot.getElevatorConfig().kPIDF());

    private TuningService<Constraints> speedTuningService =
        new SpeedConstraintsTuningService("Elevator", Robot.bot.getElevatorConfig().kConstraints());

    public Elevator(ElevatorIO elevatorIO, LimitSwitchIO limitSwitchIO) {
        this.elevatorIO = elevatorIO;
        this.limitSwitchIO = limitSwitchIO;

        profile =
            new TrapezoidProfile(
                Robot.bot.getElevatorConfig().kConstraints());
    }

    @Override
    public void periodic() {
        super.periodic();

        elevatorIO.updateInputs(elevatorInputs);
        Logger.processInputs("Elevator/Elevator", elevatorInputs);

        limitSwitchIO.updateInputs(limitSwitchInputs);
        Logger.processInputs("Elevator/LimitSwitch", limitSwitchInputs);

        // Update tunable numbers
        tuningExecutor().accept(tuningEnabled.get());

        // Reset encoder if limit switch pressed & elevator is going down
        if (limitSwitchInputs.data.pressed() && elevatorInputs.data.position() > setpoint.position) {
            elevatorIO.resetEncoder();
            setpoint = new State(0.0, 0.0);
        }

        // Update profile
        if (shouldRunProfile) {
            var goalState =
                new State(
                    range.clamp(targetHeight),
                    0.0);

            double previousVelocity = setpoint.velocity;

            setpoint =
                profile
                    .calculate(Constants.loopPeriodSecs, setpoint, goalState);

            if (!range.contains(setpoint.position)) {
                setpoint =
                    new State(
                        range.clamp(setpoint.position),
                        0.0);
            }

            atGoal = isAtHeight(goalState.position);

            if (atGoal) {
                stop();
            } else {
                double accel = (setpoint.velocity - previousVelocity) / Constants.loopPeriodSecs;
                elevatorIO.runPosition(setpoint.position, feedforward.calculate(setpoint.velocity, accel));
            }

            /// Log state
            Logger.recordOutput("Elevator/Profile/SetpointPosition", setpoint.position);
            Logger.recordOutput("Elevator/Profile/SetpointVelocity", setpoint.velocity);
            Logger.recordOutput("Elevator/Profile/GoalPosition", goalState.position);
            Logger.recordOutput("Elevator/AtGoal", atGoal);
        } else {
            // Reset setpoint
            setpoint = new State(getHeight(), 0.0);
      
            // Clear logs
            Logger.recordOutput("Elevator/Profile/SetpointPosition", 0.0);
            Logger.recordOutput("Elevator/Profile/SetpointVelocity", 0.0);
            Logger.recordOutput("Elevator/Profile/GoalPosition", 0.0);
            Logger.recordOutput("Elevator/AtGoal", true);
        }

        Logger.recordOutput("Elevator/CurrentPosition", getHeight());

        // Record cycle time
        LoggedTracer.record("Elevator");
    }

    @Override
    protected BooleanConsumer tuningExecutor() {
        return tuningEnabled -> {
            pidfTuningService.setTuning(tuningEnabled);
            speedTuningService.setTuning(tuningEnabled);
            
            if (tuningEnabled) {
                PIDFConfig newPIDFConfig = pidfTuningService.getUpdatedConfig();
                Constraints newSpeedConfig = speedTuningService.getUpdatedConfig();

                elevatorIO.setPID(
                    newPIDFConfig.kP(),
                    newPIDFConfig.kI(),
                    newPIDFConfig.kD());

                feedforward = newPIDFConfig.toElevatorFF();

                profile =
                    new TrapezoidProfile(
                        new Constraints(
                            newSpeedConfig.maxVelocity,
                            newSpeedConfig.maxAcceleration));
            }
        };
    }

    public double getHeight() {
        return elevatorInputs.data.position();
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

    public boolean isAtHeight(double setpointHeight, double tolerance) {
        return MathUtil.isNear(setpointHeight, getHeight(), tolerance);
    }

    public boolean isAtHeight(double setpointHeight) {
        return isAtHeight(setpointHeight, tolerance);
    }

    public void setGoal(ElevatorGoal goal) {
        this.shouldRunProfile = true;
        this.targetHeight = goal.getHeight();
    }
}