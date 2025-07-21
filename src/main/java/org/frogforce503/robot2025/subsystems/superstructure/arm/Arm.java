package org.frogforce503.robot2025.subsystems.superstructure.arm;

import org.frogforce503.robot2025.Constants;
import org.frogforce503.robot2025.Robot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
import org.frogforce503.lib.control.speed.SpeedConstraintsConfig;
import org.frogforce503.lib.control.speed.SpeedConstraintsTuningService;
import org.frogforce503.lib.math.MathUtils;
import org.frogforce503.lib.math.Range;
import org.frogforce503.lib.subsystem.FFSubsystemBase;
import org.frogforce503.lib.util.LoggedTracer;

public class Arm extends FFSubsystemBase {
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    // Constants
    private final Range range = Robot.bot.armConstants.range();
    private ArmFeedforward feedforward = Robot.bot.armConstants.kFF();

    // Control
    private TrapezoidProfile profile;
    @Getter private State setpoint = new State();

    // Tuning
    private TuningService<PIDFConfig> pidfTuningService =
        new PIDFTuningService("Arm",
            new PIDFConfig(
                Robot.bot.armConstants.kP(),
                Robot.bot.armConstants.kI(),
                Robot.bot.armConstants.kD(),
                Robot.bot.armConstants.kFF()));

    private TuningService<SpeedConstraintsConfig> speedTuningService =
        new SpeedConstraintsTuningService("Arm",
            new SpeedConstraintsConfig(
                Robot.bot.armConstants.maxVelocityMetersPerSec(),
                Robot.bot.armConstants.maxAccelerationMetersPerSec2()));

    // Overrides
    private LoggedNetworkBoolean tuningEnabled =
        new LoggedNetworkBoolean("Tuning/Arm/Tuning?", false);

    private LoggedNetworkBoolean coastOverride =
        new LoggedNetworkBoolean("Coast Mode/Arm", false);

    private boolean requestPositionControl = true;

    // Alerts
    private final Alert coastModeWhileRunning =
        new Alert("Arm/Warnings", "Arm is in coast mode while running!", Alert.AlertType.kWarning);

    public enum ArmGoal {
        IDLE(18),
        DOWN(18),

        PRESCORE_L1(0),
        PRESCORE_L2(0),
        PRESCORE_L3(0),
        PRESCORE_L4(0),

        SCORE_L1(0),
        SCORE_L2(0),
        SCORE_L3(0),
        SCORE_L4(1),

        POSTSCORE_L4(0),

        PLUCK_ALGAE_HIGH(0),
        PLUCK_ALGAE_LOW(0),

        HOLD_ALGAE(70),

        PROCESSOR_FROM_CLAW(70),

        NET_RELEASE(180),

        HANDOFF(130), // handoff is like arm 130 deg
        HANDOFF_RELEASE(70),

        BARGE(0);
        
        private double position;

        private ArmGoal(double position) {
            this.position = position;
        }
    }

    @Getter private ArmGoal currentGoal = ArmGoal.IDLE;

    public Arm(ArmIO io) {
        this.io = io;

        profile =
            new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                    Robot.bot.armConstants.maxVelocityMetersPerSec(), Robot.bot.armConstants.maxVelocityMetersPerSec()));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

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

            if (!MathUtils.isInRange(setpoint.position, range)) {
                setpoint =
                    new State(
                        MathUtil.clamp(setpoint.position, range.min(), range.max()),
                        0.0);
            }

            double accel = (setpoint.velocity - previousVelocity) / Constants.loopPeriodSecs;

            io.runPosition(setpoint.position, feedforward.calculate(Math.toRadians(currentGoal.position - 88.5), setpoint.velocity, accel));

            // Log state
            Logger.recordOutput("Arm/Profile/SetpointPositionMeters", setpoint.position);
            Logger.recordOutput("Arm/Profile/SetpointVelocityMetersPerSec", setpoint.velocity);
            Logger.recordOutput("Arm/Profile/GoalPositionMeters", goalState.position);
        }

        Logger.recordOutput("Arm/Goal", currentGoal.name());
        Logger.recordOutput("Arm/Reached Goal", atGoal());

        // Record cycle time
        LoggedTracer.record("Arm");
    }

    public double getPosition() {
        return inputs.data.position();
    }

    @Override
    public BooleanConsumer tuningExecutor() {
        return tuningEnabled -> {
            pidfTuningService.setTuning(tuningEnabled);
            speedTuningService.setTuning(tuningEnabled);
            
            if (tuningEnabled) {
                PIDFConfig newPIDFConfig = pidfTuningService.getUpdatedConfig();
                SpeedConstraintsConfig newSpeedConfig = speedTuningService.getUpdatedConfig();

                io.setPID(
                    newPIDFConfig.kP(),
                    newPIDFConfig.kI(),
                    newPIDFConfig.kD());

                feedforward = new ArmFeedforward(newPIDFConfig.kS(), newPIDFConfig.kG(), newPIDFConfig.kV(), newPIDFConfig.kA());

                profile =
                    new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(
                            newSpeedConfig.maxVelocityMetersPerSec(),
                            newSpeedConfig.maxAccelerationMetersPerSec2()));
            }
        };
    }

    @Override
    public boolean atGoal() {
        return MathUtil.isNear(currentGoal.position, inputs.data.position(), 2);
    }

    public void setBrakeMode(boolean enabled) {
        io.setBrakeMode(enabled);
    }

    @Override
    public Command stop() {
        return Commands.sequence(
            runOnce(() -> requestPositionControl = false),
            runOnce(io::stop)
        );
    }

    @Override
    public Command runManual(double output) {
        return Commands.sequence(
            runOnce(() -> requestPositionControl = false),
            run(() -> io.runOpenLoop(output))
        );
    }

    public Command runGoal(ArmGoal goal) {
        return Commands.sequence(
            runOnce(() -> requestPositionControl = true),
            runOnce(() -> currentGoal = goal),
            Commands.waitUntil(this::atGoal)
        );
    }
}