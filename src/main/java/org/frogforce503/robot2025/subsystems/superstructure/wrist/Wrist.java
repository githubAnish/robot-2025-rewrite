package org.frogforce503.robot2025.subsystems.superstructure.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lombok.Getter;

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
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class Wrist extends FFSubsystemBase {
    private final WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    // Constants
    private final Range range = Robot.bot.wristConstants.range();
    private ArmFeedforward feedforward = Robot.bot.wristConstants.kPIDF().toArmFeedforward();

    // Control
    private TrapezoidProfile profile;
    @Getter private State setpoint = new State();

    // Tuning
    private TuningService<PIDFConfig> pidfTuningService =
        new PIDFTuningService("Wrist", Robot.bot.wristConstants.kPIDF());

    private TuningService<Constraints> speedTuningService =
        new SpeedConstraintsTuningService("Wrist", Robot.bot.wristConstants.kConstraints());

    // Overrides
    private LoggedNetworkBoolean tuningEnabled =
        new LoggedNetworkBoolean("Tuning/Wrist/Tuning?", false);

    private LoggedNetworkBoolean coastOverride =
        new LoggedNetworkBoolean("Coast Mode/Wrist", false);

    private boolean requestPositionControl = true;

    // Alerts
    private final Alert coastModeWhileRunning =
        new Alert("Wrist/Warnings", "Wrist is in coast mode while running!", Alert.AlertType.kWarning);

    public enum WristGoal {
        TOZERO(24),
        TO90(90),

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

        INTAKE_CORAL(54),

        HOLD_ALGAE(30),

        HANDOFF(10),

        HANDOFF_RELEASE(10),

        PROCESSOR_FROM_CLAW(45),

        POSTSCORE_L4(0),

        BARGE(0),

        NET_RELEASE(230);
        
        private double position;

        private WristGoal(double position) {
            this.position = position;
        }
    }

    @Getter private WristGoal currentGoal = WristGoal.INTAKE_CORAL;

    public Wrist(WristIO io) {
        this.io = io;

        profile =
            new TrapezoidProfile(
                    Robot.bot.wristConstants.kConstraints());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Wrist", inputs);

        coastModeWhileRunning
            .set(coastOverride.get() && !RobotState.isDisabled());

        // Update tunable numbers
        tuningExecutor().accept(tuningEnabled.get());

        // Set coast mode
        if (RobotState.isDisabled()) {
            setBrakeMode(!coastOverride.get());
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

            io.runPosition(setpoint.position, feedforward.calculate(Math.toRadians(currentGoal.position - 88.5), setpoint.velocity, accel));

            // Log state
            Logger.recordOutput("Wrist/Profile/SetpointPositionMeters", setpoint.position);
            Logger.recordOutput("Wrist/Profile/SetpointVelocityMetersPerSec", setpoint.velocity);
            Logger.recordOutput("Wrist/Profile/GoalPositionMeters", goalState.position);
        }

        Logger.recordOutput("Wrist/Goal", currentGoal.name());
        Logger.recordOutput("Wrist/Reached Goal", atGoal());

        // Record cycle time
        LoggedTracer.record("Wrist");
    }

    public void setEncoderPosition(double position) {
        io.setEncoderPosition(position);
    }

    public double getPosition() {
        return inputs.data.relativePosition();
    }

    public double getAbsolutePosition() {
        return inputs.data.absolutePosition();
    }

    @Override
    public BooleanConsumer tuningExecutor() {
        return tuningEnabled -> {
            pidfTuningService.setTuning(tuningEnabled);
            speedTuningService.setTuning(tuningEnabled);
            
            if (tuningEnabled) {
                PIDFConfig newPIDFConfig = pidfTuningService.getUpdatedConfig();
                Constraints newSpeedConfig = speedTuningService.getUpdatedConfig();

                io.setPID(
                    newPIDFConfig.kP(),
                    newPIDFConfig.kI(),
                    newPIDFConfig.kD());

                feedforward = new ArmFeedforward(newPIDFConfig.kS(), newPIDFConfig.kG(), newPIDFConfig.kV(), newPIDFConfig.kA());

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
        return MathUtil.isNear(currentGoal.position, inputs.data.relativePosition(), 1);
    }

    @Override
    public Command stop() {
        return Commands.sequence(
            runOnce(() -> requestPositionControl = false),
            runOnce(io::stop)
        );
    }

    public void setBrakeMode(boolean enabled) {
        io.setBrakeMode(enabled);
    }

    @Override
    public Command runManual(double output) {
        return Commands.sequence(
            runOnce(() -> requestPositionControl = false),
            run(() -> io.runOpenLoop(output))
        );
    }

    public Command runGoal(WristGoal goal) {
        return Commands.sequence(
            runOnce(() -> requestPositionControl = true),
            runOnce(() -> currentGoal = goal),
            Commands.waitUntil(this::atGoal)
        );
    }
}