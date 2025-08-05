package org.frogforce503.robot2025.subsystems.superstructure.elevator;

import org.frogforce503.lib.control.TuningService;
import org.frogforce503.lib.control.pidf.PIDFConfig;
import org.frogforce503.lib.control.pidf.PIDFTuningService;

import org.frogforce503.lib.control.speed.SpeedConstraintsTuningService;
import org.frogforce503.lib.math.MathUtils;
import org.frogforce503.lib.math.Range;
import org.frogforce503.lib.subsystem.FFSubsystemBase;
import org.frogforce503.lib.util.LoggedTracer;
import org.frogforce503.robot2025.Constants;
import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.subsystems.superstructure.sensors.DigitalIO;
import org.frogforce503.robot2025.subsystems.superstructure.sensors.DigitalIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lombok.Getter;

public class Elevator extends FFSubsystemBase {
    private final ElevatorIO elevatorIO;
    private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
    
    private final DigitalIO digitalIO;
    private final DigitalIOInputsAutoLogged digitalInputs = new DigitalIOInputsAutoLogged();

    // Constants
    private final Range range = Robot.bot.elevatorConstants.range();
    private ElevatorFeedforward feedforward = Robot.bot.elevatorConstants.kPIDF().toElevatorFeedforward();
    
    // Control
    private TrapezoidProfile profile;
    @Getter private State setpoint = new State();

    // Tuning
    private TuningService<PIDFConfig> pidfTuningService =
        new PIDFTuningService("Elevator", Robot.bot.elevatorConstants.kPIDF());

    private TuningService<Constraints> speedTuningService =
        new SpeedConstraintsTuningService("Elevator", Robot.bot.elevatorConstants.kConstraints());

    // Overrides
    private LoggedNetworkBoolean tuningEnabled =
        new LoggedNetworkBoolean("Tuning/Elevator/Tuning?", false);

    private LoggedNetworkBoolean coastOverride =
        new LoggedNetworkBoolean("Coast Mode/Elevator", false);

    private boolean requestPositionControl = true;

    // Alerts
    private final Alert coastModeWhileRunning =
        new Alert("Elevator/Warnings", "Elevator is in coast mode while running!", Alert.AlertType.kWarning);

    public enum ElevatorGoal {
        DOWN(0),

        PRESCORE_L1(0),
        PRESCORE_L2(0),
        PRESCORE_L3(0),
        PRESCORE_L4(0),

        SCORE_L1(0),
        SCORE_L2(0),
        SCORE_L3(0),
        SCORE_L4(0),

        PLUCK_ALGAE_HIGH(0),
        PLUCK_ALGAE_LOW(0),

        HOLD_CORAL(10),
        HOLD_ALGAE(0),

        HANDOFF(17),
        HANDOFF_RELEASE(0),

        BARGE(0),

        SWITCH_MODE(20);
        
        private double position;

        private ElevatorGoal(double position) {
            this.position = position;
        }
    }

    @Getter private ElevatorGoal currentGoal = ElevatorGoal.DOWN;

    public Elevator(ElevatorIO elevatorIO, DigitalIO digitalIO) {
        this.elevatorIO = elevatorIO;
        this.digitalIO = digitalIO;

        profile =
            new TrapezoidProfile(
                    Robot.bot.elevatorConstants.kConstraints());
    }

    @Override
    public void periodic() {
        elevatorIO.updateInputs(elevatorInputs);
        Logger.processInputs("Elevator/Elevator", elevatorInputs);

        digitalIO.updateInputs(digitalInputs);
        Logger.processInputs("Elevator/LimitSwitch", digitalInputs);

        coastModeWhileRunning
            .set(coastOverride.get() && !RobotState.isDisabled());

        // Update tunable numbers
        tuningExecutor().accept(tuningEnabled.get());

        // Set coast mode
        if (RobotState.isDisabled()) {
            setBrakeMode(!coastOverride.get());
        }

        // Reset encoder if limit switch pressed & elevator is going down
        if (digitalInputs.data.pressed() && elevatorInputs.data.position() > setpoint.position) {
            elevatorIO.resetEncoder();
        }

        // Run position mode unless requested to stop
        if (requestPositionControl) {
            var goalState =
                new State(
                    MathUtil.clamp(currentGoal.position, range.min(), range.max()),
                    0.0);

            double previousVelocity = setpoint.velocity;

            setpoint =
                profile
                    .calculate(Constants.loopPeriodSecs, setpoint, goalState);

            if (!MathUtils.inRange(setpoint.position, range)) {
                setpoint =
                    new State(
                        MathUtil.clamp(setpoint.position, range.min(), range.max()),
                        0.0);
            }

            double accel = (setpoint.velocity - previousVelocity) / Constants.loopPeriodSecs;

            elevatorIO.runPosition(setpoint.position, feedforward.calculate(setpoint.velocity, accel));

            // Log state
            Logger.recordOutput("Elevator/Profile/SetpointPositionMeters", setpoint.position);
            Logger.recordOutput("Elevator/Profile/SetpointVelocityMetersPerSec", setpoint.velocity);
            Logger.recordOutput("Elevator/Profile/GoalPositionMeters", goalState.position);
        }

        Logger.recordOutput("Elevator/Goal", currentGoal.name());
        Logger.recordOutput("Elevator/Reached Goal", atGoal());

        // Record cycle time
        LoggedTracer.record("Elevator");
    }

    public double getPosition() {
        return elevatorInputs.data.position();
    }

    @Override
    public BooleanConsumer tuningExecutor() {
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

                feedforward = new ElevatorFeedforward(newPIDFConfig.kS(), newPIDFConfig.kG(), newPIDFConfig.kV(), newPIDFConfig.kA());

                profile =
                    new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(
                            newSpeedConfig.maxVelocity,
                            newSpeedConfig.maxAcceleration));
            }
        };
    }

    @Override
    public boolean atGoal() {
        return MathUtil.isNear(currentGoal.position, elevatorInputs.data.position(), 1);
    }

    public void setBrakeMode(boolean enabled) {
        elevatorIO.setBrakeMode(enabled);
    }

    @Override
    public Command stop() {
        return Commands.sequence(
            runOnce(() -> requestPositionControl = false),
            runOnce(elevatorIO::stop)
        );
    }

    @Override
    public Command runManual(double output) {
        return Commands.sequence(
            runOnce(() -> requestPositionControl = false),
            run(() -> elevatorIO.runOpenLoop(output))
        );
    }

    public Command runGoal(ElevatorGoal goal) {
        return Commands.sequence(
            runOnce(() -> requestPositionControl = true),
            runOnce(() -> currentGoal = goal),
            Commands.waitUntil(this::atGoal)
        );
    }
}